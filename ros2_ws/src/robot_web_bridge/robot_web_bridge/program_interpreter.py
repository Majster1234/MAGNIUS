"""Interpreter programu blokowego (bez ROS): przechodzi drzewo bloków i wydaje zdarzenia ruchu.

Kursor programu = blok, który interpreter właśnie przetwarza. Zdarzenia:
- ``Move``  — ruch PTP/LIN do pozycji (kąty przegubów w °) z prędkością i promieniem pominięcia punktu;
- ``Sync``  — punkt synchronizacji (zarezerwowane: czekanie na operatora/wejście; kończy ciąg ruchów).
Obsługiwane rozkazy: tablica ``HANDLERS`` (obecnie move_ptp, move_lin). Opis: docs/programming.md.
"""
import ast
import math
import operator
import re
from dataclasses import dataclass, field

MAX_MOVES = 2000
MAX_LOOP = 10000


class ProgramFailure(Exception):
    """Błąd programu przypięty do bloku (block_id może być None)."""

    def __init__(self, text, block_id=None):
        super().__init__(text)
        self.block_id = block_id


@dataclass
class Move:
    block_id: str
    kind: str                 # 'PTP' | 'LIN'
    position: str             # nazwa pozycji
    joints_deg: list
    speed: float              # PTP: % ; LIN: mm/s
    blend_mm: float           # 0 = postój w punkcie
    path: list = field(default_factory=list)   # bloki nadrzędne (pętle/warunki)


@dataclass
class Sync:
    block_id: str
    reason: str


# ----- bezpieczne wyrażenia w stylu ST (warunki przyszłych IF/WHILE) -----
_OPS = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: operator.truediv, ast.Mod: operator.mod,
        ast.Eq: operator.eq, ast.NotEq: operator.ne, ast.Lt: operator.lt, ast.LtE: operator.le, ast.Gt: operator.gt, ast.GtE: operator.ge,
        ast.USub: operator.neg, ast.UAdd: operator.pos, ast.Not: operator.not_}


def evaluate(expression, env):
    """Liczby, TRUE/FALSE, zmienne, + - * / MOD, = == <> != < <= > >=, AND OR NOT, nawiasy (bez wywołań i atrybutów)."""
    if isinstance(expression, (bool, int, float)):
        return expression
    text = (' ' + str(expression).strip() + ' ').replace('<>', '!=').replace(' AND ', ' and ').replace(' OR ', ' or ') \
        .replace(' NOT ', ' not ').replace(' MOD ', ' % ').replace('TRUE', 'True').replace('FALSE', 'False')
    tree = ast.parse(re.sub(r'(?<![=!<>])=(?!=)', '==', text).strip(), mode='eval')

    def run(node):
        if isinstance(node, ast.Expression):
            return run(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float, bool, str)):
            return node.value
        if isinstance(node, ast.Name):
            if node.id in ('True', 'False'):
                return node.id == 'True'
            if node.id not in env:
                raise ProgramFailure(f'Nieznana zmienna w wyrażeniu: {node.id}')
            return env[node.id]
        if isinstance(node, ast.BinOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](run(node.left), run(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in _OPS:
            return _OPS[type(node.op)](run(node.operand))
        if isinstance(node, ast.BoolOp):
            values = [run(v) for v in node.values]
            return all(values) if isinstance(node.op, ast.And) else any(values)
        if isinstance(node, ast.Compare):
            left = run(node.left)
            for op, right_node in zip(node.ops, node.comparators):
                right = run(right_node)
                if type(op) not in _OPS or not _OPS[type(op)](left, right):
                    return False
                left = right
            return True
        raise ProgramFailure(f'Niedozwolona konstrukcja w wyrażeniu: {expression!r}')
    return run(tree)


class Interpreter:
    def __init__(self, document):
        self.document = document
        self.positions = {p['name']: list(p['joints_deg']) for p in document.get('positions', [])}
        self.types = {v['name']: v.get('type') for v in document.get('variables', [])}
        self.env = {v['name']: v.get('init') for v in document.get('variables', [])}
        self.moves = 0
        self.cursor = None          # kursor programu: blok przetwarzany przez interpreter

    # ----- wartości parametrów -----
    def value(self, raw, block_id):
        if isinstance(raw, dict) and 'var' in raw:
            if raw['var'] not in self.env:
                raise ProgramFailure(f'Brak zmiennej „{raw["var"]}”', block_id)
            return self.env[raw['var']]
        if raw is None or raw == '':
            raise ProgramFailure('Nieuzupełnione pole bloku', block_id)
        return raw

    def position(self, raw, block_id):
        if isinstance(raw, dict) and 'position' in raw:
            name = raw['position']
        else:
            name = self.value(raw, block_id)
        if not isinstance(name, str) or name not in self.positions:
            raise ProgramFailure(f'Brak pozycji „{name}”', block_id)
        return name, self.positions[name]

    def number(self, raw, label, block_id):
        value = self.value(raw, block_id)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ProgramFailure(f'{label}: wymagana liczba, jest {value!r}', block_id)
        return float(value)

    # ----- rozkazy -----
    def move(self, block, path, kind):
        params, block_id = block.get('params', {}), block['id']
        name, joints = self.position(params.get('position'), block_id)
        speed = self.number(params.get('speed'), 'prędkość', block_id)
        blend = self.number(params.get('blend_mm', 0), 'promień pominięcia', block_id)
        if blend < 0:
            raise ProgramFailure('Promień pominięcia punktu nie może być ujemny', block_id)
        self.moves += 1
        if self.moves > MAX_MOVES:
            raise ProgramFailure(f'Program przekracza {MAX_MOVES} ruchów', block_id)
        yield Move(block_id, kind, name, joints, speed, blend, list(path))

    HANDLERS = {
        'move_ptp': lambda self, block, path: self.move(block, path, 'PTP'),
        'move_lin': lambda self, block, path: self.move(block, path, 'LIN'),
    }

    def events(self):
        yield from self.run(self.document.get('blocks', []), [])
        self.cursor = None

    def run(self, blocks, path):
        for block in blocks:
            if block.get('disabled'):
                continue
            self.cursor = block['id']
            handler = self.HANDLERS.get(block.get('command'))
            if handler is None:
                raise ProgramFailure(f'Rozkaz „{block.get("command")}” nie jest obsługiwany przez wykonawcę', block['id'])
            yield from handler(self, block, path)


def moves_of(document):
    """Wszystkie ruchy programu (bez logiki zależnej od robota) — do testów i podglądu."""
    return [event for event in Interpreter(document).events() if isinstance(event, Move)]
