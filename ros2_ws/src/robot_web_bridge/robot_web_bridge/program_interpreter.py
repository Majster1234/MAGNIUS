"""Interpreter programu blokowego (bez ROS): przechodzi drzewo bloków i wydaje zdarzenia ruchu.

Kursor programu = blok, który interpreter właśnie przetwarza. Zdarzenia:
- ``Move``  — ruch PTP/LIN/CIRC do pozycji (kąty przegubów w °) z prędkością i promieniem pominięcia punktu (CIRC: łuk
  przez pozycję pośrednią ``via``); ruch względny
  (``relative``: przesunięcie/obrót TCP w układzie bazy lub narzędzia) ma ``joints_deg=None`` — cel wyznacza wykonawca
  (FK celu poprzedniego ruchu → przyrost pozy → IK, ``ProgramRunner.resolve``);
- ``Sync``  — punkt synchronizacji kończący ciąg ruchów: ``kind`` = 'wait' (czekanie ``seconds``), 'operator'
  (czekanie na „Kontynuuj” operatora) albo 'sync' (zarezerwowane).
Logika (warunki, pętle, skoki, zmienne) liczona jest w interpreterze — bez wejść/wyjść jest deterministyczna, więc
sprawdzenie programu symuluje ją w całości. Rozkazy: tablica ``HANDLERS``. Opis: docs/programming.md.
"""
import math
import operator
import re
from dataclasses import dataclass, field

MAX_MOVES = 2000
MAX_STEPS = 100000           # wykonane bloki logiki (ochrona przed pętlą bez końca)
MAX_WAIT_S = 3600.
MAX_MAGNITUDE = 1e12         # zakres wartości liczbowych w wyrażeniach
LABEL = re.compile(r'^[^\W\d]\w{0,39}$')

LOOP_COMMANDS = ('for', 'repeat', 'while', 'repeat_until')
LOOP_JUMPS = ('break', 'continue')


class ProgramFailure(Exception):
    """Błąd programu przypięty do bloku (block_id może być None)."""

    def __init__(self, text, block_id=None):
        super().__init__(text)
        self.block_id = block_id


@dataclass
class Move:
    block_id: str
    kind: str                 # 'PTP' | 'LIN' | 'CIRC'
    position: str             # nazwa pozycji
    joints_deg: list
    speed: float              # PTP: % ; LIN/CIRC: mm/s
    blend_mm: float           # 0 = postój w punkcie
    path: list = field(default_factory=list)   # bloki nadrzędne (pętle/warunki)
    relative: dict = None     # {'frame': 'base'|'tool', 'translation_mm': [3], 'rotation_deg': [3]} — ruch względny
    angular_speed: float = None   # obrót względny: prędkość kątowa TCP [°/s]
    via: str = None           # CIRC: nazwa pozycji pośredniej łuku
    via_joints_deg: list = None   # CIRC: kąty przegubów pozycji pośredniej [°]
    circ_orientation: int = 0     # CIRC: 0 interpolacja start→cel, 1 stała względem środka okręgu


@dataclass
class Sync:
    block_id: str
    reason: str
    kind: str = 'sync'        # 'wait' | 'operator' | 'sync'
    seconds: float = 0.


# ----- sterowanie przepływem (wyjątki wewnętrzne interpretera) -----
class _Break(Exception):
    def __init__(self, block_id):
        self.block_id = block_id


class _Continue(_Break):
    pass


class _Jump(Exception):
    def __init__(self, label, block_id):
        self.label, self.block_id = label, block_id


class _Halt(Exception):
    pass


# ----- bezpieczne wyrażenia w stylu ST (własny parser — bez eval/ast Pythona) -----
def _number(x):
    if isinstance(x, bool) or not isinstance(x, (int, float)):
        raise TypeError('wymagana liczba')
    return x


def _boolean(x, what):
    if type(x) is not bool:
        raise TypeError(f'{what} wymaga wartości BOOL')
    return x


def _deg(function):
    return lambda x: function(math.radians(_number(x)))


def _inv_deg(function):
    return lambda x: math.degrees(function(_number(x)))


def _round(x):
    return int(math.floor(_number(x) + .5))


def _limit(low, x, high):
    return min(max(_number(x), _number(low)), _number(high))


FUNCTIONS = {   # nazwa: (funkcja, liczba argumentów; None = 1 lub więcej)
    'ABS': (lambda x: abs(_number(x)), 1), 'SQRT': (lambda x: math.sqrt(_number(x)), 1),
    'SIN': (_deg(math.sin), 1), 'COS': (_deg(math.cos), 1), 'TAN': (_deg(math.tan), 1),
    'ASIN': (_inv_deg(math.asin), 1), 'ACOS': (_inv_deg(math.acos), 1), 'ATAN': (_inv_deg(math.atan), 1),
    'ATAN2': (lambda y, x: math.degrees(math.atan2(_number(y), _number(x))), 2),
    'ROUND': (_round, 1), 'TRUNC': (lambda x: int(_number(x)), 1), 'FLOOR': (lambda x: math.floor(_number(x)), 1),
    'CEIL': (lambda x: math.ceil(_number(x)), 1), 'EXP': (lambda x: math.exp(_number(x)), 1), 'LN': (lambda x: math.log(_number(x)), 1),
    'LOG': (lambda x: math.log10(_number(x)), 1), 'MIN': (lambda *a: min(_number(x) for x in a), None),
    'MAX': (lambda *a: max(_number(x) for x in a), None), 'LIMIT': (_limit, 3),
}
CONSTANTS = {'PI': math.pi, 'TRUE': True, 'FALSE': False}
KEYWORDS = ('AND', 'OR', 'XOR', 'NOT', 'MOD')
_TOKEN = re.compile(r"\s*(?:(\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?)|'([^']*)'|\"([^\"]*)\"|([^\W\d]\w*)"
                    r"|(<>|<=|>=|==|!=|\*\*|[-+*/^()=<>,]))")
_COMPARE = {'=': operator.eq, '==': operator.eq, '<>': operator.ne, '!=': operator.ne, '<': operator.lt, '<=': operator.le,
            '>': operator.gt, '>=': operator.ge}
_ARITH = {'+': operator.add, '-': operator.sub, '*': operator.mul, '/': operator.truediv, 'MOD': operator.mod, '^': operator.pow}


def _tokens(text):
    out, position, end = [], 0, len(text.rstrip())
    while position < end:
        match = _TOKEN.match(text, position)
        if not match or match.end() == position:
            raise ProgramFailure(f'Niedozwolony znak w wyrażeniu „{text}” (pozycja {position + 1})')
        number, single, double, name, symbol = match.groups()
        if number is not None:
            out.append(('num', float(number) if any(c in number for c in '.eE') else int(number)))
        elif single is not None or double is not None:
            out.append(('str', single if single is not None else double))
        elif name is not None:
            out.append(('kw', name.upper()) if name.upper() in KEYWORDS else ('name', name))
        else:
            out.append(('op', {'**': '^'}.get(symbol, symbol)))
        position = match.end()
    return out


def compile_expression(expression, names):
    """Tekst → drzewo wyrażenia (krotki). Sprawdza składnię i nazwy, bez liczenia.

    Precedencja (od najsłabszej): OR, XOR, AND, NOT, porównania (= <> < <= > >=, bez łańcuchów), + -, * / MOD,
    minus jednoargumentowy, ^ (potęga, prawostronna). Literały: liczby, 'tekst', TRUE/FALSE, PI; zmienne; funkcje
    z ``FUNCTIONS`` (kąty w stopniach). Słowa kluczowe, stałe i funkcje bez względu na wielkość liter.
    """
    text = str(expression)
    if len(text) > 200:
        raise ProgramFailure('Wyrażenie dłuższe niż 200 znaków')
    tokens, index = _tokens(text), [0]
    if not tokens:
        raise ProgramFailure('Puste wyrażenie')

    def peek():
        return tokens[index[0]] if index[0] < len(tokens) else ('end', None)

    def take(kind=None, value=None):
        token = peek()
        if (kind and token[0] != kind) or (value is not None and token[1] != value):
            found = 'koniec wyrażenia' if token[0] == 'end' else f'„{token[1]}”'
            raise ProgramFailure(f'Błąd składni wyrażenia „{text}”: nieoczekiwany {found}')
        index[0] += 1
        return token

    def binary(next_level, operators):
        def parse():
            node = next_level()
            while peek()[1] in operators and peek()[0] in ('kw', 'op'):
                op = take()[1]
                node = ('bin', op, node, next_level())
            return node
        return parse

    def unary_not():
        if peek() == ('kw', 'NOT'):
            take()
            return ('not', unary_not())
        return compare()

    def compare():
        node = additive()
        if peek()[0] == 'op' and peek()[1] in _COMPARE:
            op = take()[1]
            node = ('cmp', op, node, additive())
            if peek()[0] == 'op' and peek()[1] in _COMPARE:
                raise ProgramFailure(f'Błąd składni wyrażenia „{text}”: porównania łącz przez AND')
        return node

    def unary_minus():
        if peek()[0] == 'op' and peek()[1] in ('-', '+'):
            sign = take()[1]
            return ('neg', unary_minus()) if sign == '-' else ('pos', unary_minus())
        return power()

    def power():
        node = primary()
        if peek() == ('op', '^'):
            take()
            return ('bin', '^', node, unary_minus())
        return node

    def primary():
        token = peek()
        if token[0] in ('num', 'str'):
            take()
            return token
        if token == ('op', '('):
            take()
            node = expression_()
            take('op', ')')
            return node
        if token[0] == 'name':
            take()
            name, upper = token[1], token[1].upper()
            if peek() == ('op', '('):
                if upper not in FUNCTIONS:
                    raise ProgramFailure(f'Nieznana funkcja w wyrażeniu: {name}')
                take()
                args = [] if peek() == ('op', ')') else [expression_()]
                while peek() == ('op', ','):
                    take()
                    args.append(expression_())
                take('op', ')')
                arity = FUNCTIONS[upper][1]
                if (arity is None and not args) or (arity is not None and len(args) != arity):
                    raise ProgramFailure(f'{upper}: wymagane argumenty: {arity or "co najmniej 1"}')
                return ('call', upper, args)
            if name in names:
                return ('var', name)
            if upper in CONSTANTS:
                return ('const', upper)
            raise ProgramFailure(f'Nieznana zmienna w wyrażeniu: {name}')
        found = 'koniec wyrażenia' if token[0] == 'end' else f'„{token[1]}”'
        raise ProgramFailure(f'Błąd składni wyrażenia „{text}”: nieoczekiwany {found}')

    multiplicative = binary(unary_minus, ('*', '/', 'MOD'))
    additive = binary(multiplicative, ('+', '-'))
    conjunction = binary(unary_not, ('AND',))
    exclusive = binary(conjunction, ('XOR',))
    expression_ = binary(exclusive, ('OR',))
    tree = expression_()
    if index[0] != len(tokens):
        take('end')
    return tree


def _check_range(value):
    if isinstance(value, float) and not math.isfinite(value):
        raise ProgramFailure('Wynik wyrażenia nie jest skończoną liczbą')
    if isinstance(value, (int, float)) and not isinstance(value, bool) and abs(value) > MAX_MAGNITUDE:
        raise ProgramFailure('Wynik wyrażenia poza zakresem ±1e12')
    return value


def evaluate(expression, env):
    """Wartość wyrażenia dla zmiennych ``env`` (literał bool/int/float zwracany bez zmian)."""
    if isinstance(expression, (bool, int, float)):
        return expression
    tree = compile_expression(expression, env)

    def run(node):
        kind = node[0]
        if kind in ('num', 'str'):
            return node[1]
        if kind == 'var':
            return env[node[1]]
        if kind == 'const':
            return CONSTANTS[node[1]]
        if kind == 'not':
            return not _boolean(run(node[1]), 'NOT')
        if kind == 'neg':
            return -_number(run(node[1]))
        if kind == 'pos':
            return _number(run(node[1]))
        if kind == 'call':
            return _check_range(FUNCTIONS[node[1]][0](*[run(a) for a in node[2]]))
        op, left = node[1], run(node[2])
        if op in ('AND', 'OR'):   # skrócone wartościowanie
            if _boolean(left, op) == (op == 'OR'):
                return left
            return _boolean(run(node[3]), op)
        right = run(node[3])
        if kind == 'cmp':
            return _COMPARE[op](left, right)
        if op == 'XOR':
            return _boolean(left, op) != _boolean(right, op)
        if op == '+' and isinstance(left, str) and isinstance(right, str):
            return left + right
        _number(left), _number(right)
        if op == '^' and abs(right) > 100:
            raise ProgramFailure('Wykładnik potęgi poza zakresem ±100')
        return _check_range(_ARITH[op](left, right))

    try:
        return run(tree)
    except ProgramFailure:
        raise
    except ZeroDivisionError:
        raise ProgramFailure(f'Dzielenie przez zero w wyrażeniu „{expression}”') from None
    except (TypeError, ValueError, OverflowError) as error:
        raise ProgramFailure(f'Nie można obliczyć „{expression}”: {error}') from None


def _fmt(value):
    return ('TRUE' if value else 'FALSE') if isinstance(value, bool) else f'{value:g}' if isinstance(value, float) else str(value)


class Interpreter:
    def __init__(self, document):
        self.document = document
        self.positions = {p['name']: list(p['joints_deg']) for p in document.get('positions', [])}
        self.variables = {v['name']: v for v in document.get('variables', [])}
        self.types = {v['name']: v.get('type') for v in document.get('variables', [])}
        self.env = {v['name']: v.get('init') for v in document.get('variables', [])}
        self.moves = 0
        self.steps = 0
        self.cursor = None          # kursor programu: blok przetwarzany przez interpreter

    # ----- wartości parametrów -----
    def value(self, raw, block_id):
        if isinstance(raw, dict) and 'var' in raw:
            if raw['var'] not in self.env:
                raise ProgramFailure(f'Brak zmiennej „{raw["var"]}”', block_id)
            return self.env[raw['var']]
        if isinstance(raw, dict) and 'position' in raw:
            if raw['position'] not in self.positions:
                raise ProgramFailure(f'Brak pozycji „{raw["position"]}”', block_id)
            return raw['position']
        if raw is None or raw == '':
            raise ProgramFailure('Nieuzupełnione pole bloku', block_id)
        return raw

    def position(self, raw, block_id):
        name = self.value(raw, block_id)
        if not isinstance(name, str) or name not in self.positions:
            raise ProgramFailure(f'Brak pozycji „{name}”', block_id)
        return name, self.positions[name]

    def number(self, raw, label, block_id):
        value = self.value(raw, block_id)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ProgramFailure(f'{label}: wymagana liczba, jest {value!r}', block_id)
        return float(value)

    def integer(self, raw, label, block_id):
        value = self.number(raw, label, block_id)
        if value != int(value):
            raise ProgramFailure(f'{label}: wymagana liczba całkowita, jest {value:g}', block_id)
        return int(value)

    def expression(self, raw, block_id):
        """Parametr-wyrażenie: odwołanie do zmiennej/pozycji, literał liczby/BOOL albo tekst wyrażenia."""
        value = self.value(raw, block_id)
        if isinstance(raw, dict) or not isinstance(value, str):
            return value
        try:
            return evaluate(value, self.env)
        except ProgramFailure as error:
            raise ProgramFailure(str(error), block_id) from None

    def condition(self, raw, block_id):
        value = self.expression(raw, block_id)
        if type(value) is not bool:
            raise ProgramFailure(f'Warunek musi mieć wartość BOOL (TRUE/FALSE), jest {_fmt(value)}', block_id)
        return value

    def target(self, raw, block_id, types=None):
        if not (isinstance(raw, dict) and 'var' in raw):
            raise ProgramFailure('Wymagana zmienna docelowa', block_id)
        name = raw['var']
        if name not in self.env:
            raise ProgramFailure(f'Brak zmiennej „{name}”', block_id)
        if types and self.types[name] not in types:
            raise ProgramFailure(f'Zmienna {name} ({self.types[name]}): wymagany typ {"/".join(types)}', block_id)
        return name

    def assign(self, name, value, block_id):
        """Przypisanie z konwersją do typu zmiennej (INT: zaokrąglenie) i kontrolą zakresu min/max zmiennej."""
        kind, label = self.types[name], f'Zmienna {name} ({self.types[name]})'
        if kind in ('INT', 'REAL'):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ProgramFailure(f'{label}: wymagana liczba, jest {_fmt(value)}', block_id)
            value = _round(value) if kind == 'INT' else float(value)
            variable = self.variables[name]
            low, high = variable.get('min'), variable.get('max')
            if (low is not None and value < low) or (high is not None and value > high):
                raise ProgramFailure(f'{label}: wartość {_fmt(value)} poza zakresem zmiennej '
                                     f'[{"–" if low is None else _fmt(low)} … {"–" if high is None else _fmt(high)}]', block_id)
        elif kind == 'BOOL':
            if type(value) is not bool:
                raise ProgramFailure(f'{label}: wymagana wartość TRUE/FALSE, jest {_fmt(value)}', block_id)
        elif kind == 'POZYCJA':
            if not isinstance(value, str) or value not in self.positions:
                raise ProgramFailure(f'{label}: brak pozycji „{_fmt(value)}”', block_id)
        else:
            value = value if isinstance(value, str) else _fmt(value)
        self.env[name] = value

    # ----- rozkazy: ruch i czas -----
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

    def move_circ(self, block, path):
        """Łuk okręgu TCP od celu poprzedniego ruchu przez pozycję pośrednią do pozycji docelowej (prędkość TCP mm/s)."""
        params = block.get('params', {})
        via, via_joints = self.position(params.get('via'), block['id'])
        orientation = self.CIRC_ORIENTATIONS.get(params.get('orientation', 'interpolowana'))
        if orientation is None:
            raise ProgramFailure(f'Orientacja CIRC: interpolowana albo względem środka, jest {params.get("orientation")!r}', block['id'])
        for move in self.move(block, path, 'CIRC'):
            move.via, move.via_joints_deg, move.circ_orientation = via, via_joints, orientation
            yield move

    CIRC_ORIENTATIONS = {'interpolowana': 0, 'względem środka': 1}

    FRAMES = {'baza': 'base', 'narzędzie': 'tool'}

    def move_relative(self, block, path, rotation):
        """Przesunięcie (mm) albo obrót (°, kolejno wokół X, Y, Z) TCP względem celu poprzedniego ruchu; ruch liniowy."""
        params, block_id = block.get('params', {}), block['id']
        frame = self.FRAMES.get(params.get('frame'))
        if frame is None:
            raise ProgramFailure(f'Układ ruchu względnego: baza albo narzędzie, jest {params.get("frame")!r}', block_id)
        names, unit = (('rx', 'ry', 'rz'), '°') if rotation else (('dx', 'dy', 'dz'), 'mm')
        values = [self.number(params.get(name), name.upper(), block_id) for name in names]
        if not any(values):
            raise ProgramFailure('Ruch względny: wszystkie przyrosty są zerowe', block_id)
        speed = self.number(params.get('speed'), 'prędkość', block_id)
        blend = self.number(params.get('blend_mm', 0), 'promień pominięcia', block_id)
        if blend < 0:
            raise ProgramFailure('Promień pominięcia punktu nie może być ujemny', block_id)
        self.moves += 1
        if self.moves > MAX_MOVES:
            raise ProgramFailure(f'Program przekracza {MAX_MOVES} ruchów', block_id)
        label = f'Δ{"R" if rotation else ""}({", ".join(_fmt(v) for v in values)}) {unit} {params.get("frame")}'
        relative = {'frame': frame, 'translation_mm': [0., 0., 0.] if rotation else values, 'rotation_deg': values if rotation else [0., 0., 0.]}
        yield Move(block_id, 'LIN', label, None, 0. if rotation else speed, blend, list(path), relative, speed if rotation else None)

    def wait_time(self, block, path):
        seconds = self.number(block.get('params', {}).get('seconds'), 'czas', block['id'])
        if not 0 <= seconds <= MAX_WAIT_S:
            raise ProgramFailure(f'Czas czekania {seconds:g} s poza zakresem 0–{MAX_WAIT_S:g} s', block['id'])
        yield Sync(block['id'], f'Czekanie {seconds:g} s', 'wait', seconds)

    def wait_operator(self, block, path):
        message = block.get('params', {}).get('message')
        yield Sync(block['id'], message if isinstance(message, str) and message.strip() else 'Czekanie na operatora', 'operator')

    # ----- rozkazy: logika -----
    def if_block(self, block, path):
        branch = 'body' if self.condition(block.get('params', {}).get('condition'), block['id']) else 'else'
        yield from self.run(block.get(branch, []), path + [block['id']])

    def switch(self, block, path):
        value = self.expression(block.get('params', {}).get('value'), block['id'])
        for case in block.get('body', []):
            if case.get('disabled') or case.get('command') != 'case':
                continue
            self.step(case)
            if self.value(case.get('params', {}).get('match'), case['id']) == value:
                yield from self.run(case.get('body', []), path + [block['id'], case['id']])
                return
        yield from self.run(block.get('else', []), path + [block['id']])

    def logic_set(self, block, path):
        params = block.get('params', {})
        name = self.target(params.get('target'), block['id'], ('BOOL',))
        self.assign(name, self.condition(params.get('condition'), block['id']), block['id'])
        yield from ()

    def logic_toggle(self, block, path):
        name = self.target(block.get('params', {}).get('target'), block['id'], ('BOOL',))
        self.assign(name, not self.env[name], block['id'])
        yield from ()

    # ----- rozkazy: matematyka -----
    def assign_block(self, block, path):
        params = block.get('params', {})
        name = self.target(params.get('target'), block['id'])
        self.assign(name, self.expression(params.get('expression'), block['id']), block['id'])
        yield from ()

    def increment(self, block, path, sign):
        params = block.get('params', {})
        name = self.target(params.get('target'), block['id'], ('INT', 'REAL'))
        self.assign(name, self.env[name] + sign * self.number(params.get('step'), 'krok', block['id']), block['id'])
        yield from ()

    def calc(self, block, path):
        params, block_id = block.get('params', {}), block['id']
        name = self.target(params.get('target'), block_id, ('INT', 'REAL'))
        a, b, op = self.number(params.get('a'), 'a', block_id), self.number(params.get('b'), 'b', block_id), params.get('op')
        if op in ('/', 'MOD') and b == 0:
            raise ProgramFailure('Dzielenie przez zero', block_id)
        operations = {'+': operator.add, '-': operator.sub, '*': operator.mul, '/': operator.truediv, 'MOD': operator.mod,
                      '^': operator.pow, 'MIN': min, 'MAX': max}
        if op not in operations:
            raise ProgramFailure(f'Nieznane działanie {op!r}', block_id)
        if op == '^' and abs(b) > 100:
            raise ProgramFailure('Wykładnik potęgi poza zakresem ±100', block_id)
        try:
            result = _check_range(operations[op](a, b))
        except (OverflowError, ValueError, TypeError) as error:
            raise ProgramFailure(f'Nie można obliczyć: {error}', block_id) from None
        except ProgramFailure as error:
            raise ProgramFailure(str(error), block_id) from None
        self.assign(name, result, block_id)
        yield from ()

    def math_func(self, block, path):
        params, block_id = block.get('params', {}), block['id']
        name = self.target(params.get('target'), block_id, ('INT', 'REAL'))
        func = params.get('func')
        if func not in FUNCTIONS or func in ('MIN', 'MAX', 'LIMIT', 'ATAN2'):
            raise ProgramFailure(f'Nieznana funkcja {func!r}', block_id)
        try:
            result = _check_range(FUNCTIONS[func][0](self.number(params.get('arg'), 'argument', block_id)))
        except (ValueError, OverflowError) as error:
            raise ProgramFailure(f'{func}: argument poza dziedziną ({error})', block_id) from None
        except ProgramFailure as error:
            raise ProgramFailure(str(error), error.block_id or block_id) from None
        self.assign(name, result, block_id)
        yield from ()

    def limit(self, block, path):
        params, block_id = block.get('params', {}), block['id']
        name = self.target(params.get('target'), block_id, ('INT', 'REAL'))
        low, high = self.number(params.get('min'), 'min', block_id), self.number(params.get('max'), 'maks', block_id)
        if low > high:
            raise ProgramFailure(f'Ograniczenie: min {low:g} większe od maks {high:g}', block_id)
        self.assign(name, _limit(low, self.env[name], high), block_id)
        yield from ()

    # ----- rozkazy: pętle i skoki -----
    def loop_body(self, block, path):
        """Jeden obieg pętli → False, gdy przerwano (break)."""
        try:
            yield from self.run(block.get('body', []), path + [block['id']])
        except _Continue:
            pass
        except _Break:
            return False
        return True

    def for_block(self, block, path):
        params, block_id = block.get('params', {}), block['id']
        counter = self.target(params.get('counter'), block_id, ('INT',))
        start, stop = self.integer(params.get('from'), 'od', block_id), self.integer(params.get('to'), 'do', block_id)
        step = self.integer(params.get('step', 1), 'krok', block_id)
        if step == 0:
            raise ProgramFailure('Pętla: krok nie może być 0', block_id)
        value = start
        while (value <= stop) if step > 0 else (value >= stop):
            self.assign(counter, value, block_id)
            if not (yield from self.loop_body(block, path)):
                return
            self.step(block)
            value = self.env[counter] + step   # po pętli licznik zachowuje ostatnią wartość z zakresu

    def repeat(self, block, path):
        count = self.integer(block.get('params', {}).get('count'), 'liczba powtórzeń', block['id'])
        for _ in range(max(0, count)):
            if not (yield from self.loop_body(block, path)):
                return
            self.step(block)

    def while_block(self, block, path):
        while self.condition(block.get('params', {}).get('condition'), block['id']):
            if not (yield from self.loop_body(block, path)):
                return
            self.step(block)

    def repeat_until(self, block, path):
        while True:
            if not (yield from self.loop_body(block, path)):
                return
            self.step(block)
            self.cursor = block['id']
            if self.condition(block.get('params', {}).get('condition'), block['id']):
                return

    def jump(self, block, path, kind):
        if kind == 'break':
            raise _Break(block['id'])
        if kind == 'continue':
            raise _Continue(block['id'])
        if kind == 'stop':
            raise _Halt()
        raise _Jump(block.get('params', {}).get('target'), block['id'])
        yield  # pragma: no cover (generator)

    def nothing(self, block, path):
        yield from ()

    def case_outside(self, block, path):
        raise ProgramFailure('„Przypadek” tylko wewnątrz bloku „Wybierz”', block['id'])
        yield  # pragma: no cover (generator)

    HANDLERS = {
        'move_ptp': lambda self, block, path: self.move(block, path, 'PTP'),
        'move_lin': lambda self, block, path: self.move(block, path, 'LIN'),
        'move_circ': move_circ,
        'move_rel': lambda self, block, path: self.move_relative(block, path, False),
        'rotate_rel': lambda self, block, path: self.move_relative(block, path, True),
        'wait_time': wait_time, 'wait_operator': wait_operator,
        'if': if_block, 'switch': switch, 'logic_set': logic_set, 'logic_toggle': logic_toggle,
        'assign': assign_block, 'calc': calc, 'math_func': math_func, 'limit': limit,
        'increment': lambda self, block, path: self.increment(block, path, 1),
        'decrement': lambda self, block, path: self.increment(block, path, -1),
        'for': for_block, 'repeat': repeat, 'while': while_block, 'repeat_until': repeat_until,
        'break': lambda self, block, path: self.jump(block, path, 'break'),
        'continue': lambda self, block, path: self.jump(block, path, 'continue'),
        'goto': lambda self, block, path: self.jump(block, path, 'goto'),
        'stop_program': lambda self, block, path: self.jump(block, path, 'stop'),
        'label': nothing, 'case': case_outside,
    }

    def step(self, block):
        self.steps += 1
        if self.steps > MAX_STEPS:
            raise ProgramFailure(f'Program przekracza {MAX_STEPS} kroków logiki (pętla bez końca?)', block['id'])

    def events(self):
        try:
            yield from self.run(self.document.get('blocks', []), [])
        except _Halt:
            pass
        except _Break as signal:
            raise ProgramFailure('„Przerwij”/„Następny obieg” poza pętlą', signal.block_id) from None
        except _Jump as signal:
            raise ProgramFailure(f'Skok do niedostępnej etykiety „{signal.label}”', signal.block_id) from None
        self.cursor = None

    def run(self, blocks, path):
        labels = {b.get('params', {}).get('name'): k for k, b in enumerate(blocks) if b.get('command') == 'label' and not b.get('disabled')}
        index = 0
        while index < len(blocks):
            block = blocks[index]
            index += 1
            if block.get('disabled'):
                continue
            self.step(block)
            self.cursor = block['id']
            handler = self.HANDLERS.get(block.get('command'))
            if handler is None:
                raise ProgramFailure(f'Rozkaz „{block.get("command")}” nie jest obsługiwany przez wykonawcę', block['id'])
            try:
                yield from handler(self, block, path)
            except _Jump as signal:
                if signal.label not in labels:
                    raise
                index = labels[signal.label]


def moves_of(document):
    """Wszystkie ruchy programu (bez logiki zależnej od robota) — do testów i podglądu."""
    return [event for event in Interpreter(document).events() if isinstance(event, Move)]


# ----- statyczne sprawdzenie struktury (uwagi, nie błędy zapisu) -----
def flow_problems(document, commands):
    """Etykiety i skoki, przerwij/następny obieg poza pętlą, wyrażenia, typy zmiennych, krok pętli, wartości z listy."""
    out, names = [], set(v['name'] for v in document.get('variables', []))
    types = {v['name']: v.get('type') for v in document.get('variables', [])}
    labels = {}

    def collect(blocks):
        for block in blocks:
            if block.get('disabled'):
                continue
            if block.get('command') == 'label':
                name = block.get('params', {}).get('name')
                if isinstance(name, str) and name:
                    labels[name] = labels.get(name, 0) + 1
            for key, child in block.items():
                if isinstance(child, list):
                    collect(child)
    collect(document.get('blocks', []))
    out += [f'Etykieta „{name}” występuje {count}×' for name, count in sorted(labels.items()) if count > 1]

    def visit(blocks, scope, in_loop):
        here = scope + [{b.get('params', {}).get('name') for b in blocks if b.get('command') == 'label' and not b.get('disabled')}]
        for block in blocks:
            if block.get('disabled'):
                continue
            command, params, prefix = commands.get(block.get('command')) or {}, block.get('params', {}), f'Blok {block["id"]}'
            cid = block.get('command')
            if cid == 'label' and isinstance(params.get('name'), str) and params['name'] and not LABEL.match(params['name']):
                out.append(f'{prefix}: nazwa etykiety — litera, dalej litery, cyfry lub _ (do 40 znaków)')
            if cid == 'goto' and isinstance(params.get('target'), str) and params['target']:
                if params['target'] not in labels:
                    out.append(f'{prefix}: skok do nieistniejącej etykiety „{params["target"]}”')
                elif not any(params['target'] in level for level in here):
                    out.append(f'{prefix}: etykieta „{params["target"]}” niedostępna — skok tylko w tej samej lub nadrzędnej liście bloków')
            if cid in LOOP_JUMPS and not in_loop:
                out.append(f'{prefix}: „{command.get("name", cid)}” poza pętlą')
            if cid == 'for' and params.get('step') == 0:
                out.append(f'{prefix}: krok pętli nie może być 0')
            for param in command.get('params', []):
                value, label = params.get(param['name']), f'{prefix}: {param.get("label", param["name"])}'
                if param.get('kind') in ('condition', 'expression') and isinstance(value, str) and value.strip():
                    try:
                        compile_expression(value, names)
                    except ProgramFailure as error:
                        out.append(f'{label} — {error}')
                if param.get('types') and isinstance(value, dict) and value.get('var') in types and types[value['var']] not in param['types']:
                    out.append(f'{label} — zmienna {value["var"]} ({types[value["var"]]}), wymagany typ {"/".join(param["types"])}')
                if param.get('options') and isinstance(value, str) and value and value not in param['options']:
                    out.append(f'{label} — „{value}” spoza listy ({", ".join(param["options"])})')
            loop = in_loop or cid in LOOP_COMMANDS
            for slot in command.get('slots', []):
                visit(block.get(slot['name'], []), here, loop)
    visit(document.get('blocks', []), [], False)
    return out
