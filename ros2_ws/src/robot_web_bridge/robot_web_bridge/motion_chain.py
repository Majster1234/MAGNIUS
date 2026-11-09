"""Ciągi ruchów (bez ROS): limity z konfiguracji, podział programu na ciągi i kursor robota.

Ciąg = ruchy wysyłane do wykonawcy jako jedna trajektoria (planer: ComputeMotionChain, motion_type "CHAIN").
Wewnątrz ciągu promień 0 oznacza postój w punkcie (v=0), promień > 0 — płynne przejście. Ciąg kończy się na końcu
programu, punkcie synchronizacji albo limicie (czas, liczba ruchów); ostatni ruch ciągu zawsze się zatrzymuje.
"""
import math
from collections import deque
from .program_interpreter import Move, ProgramFailure

MAX_CHAIN_SEGMENTS = 64
CHAIN_ESTIMATE_S = 80.        # szacunek dzielący ciągi (planer ucina twardo na max_duration)
CHAIN_MAX_DURATION_S = 95.    # limit planera dla ciągu (wykonawca: 10 000 punktów / 100 s przy 10 ms)
BLEND_MAX_MM = 500.


class MotionLimits:
    """Limity osi i TCP z aktywnej konfiguracji robota (istniejące klucze; nic nie jest zgadywane)."""

    def __init__(self, joint_velocity, joint_acceleration, tcp_velocity, tcp_acceleration, tcp_angular):
        self.joint_velocity, self.joint_acceleration = list(joint_velocity), list(joint_acceleration)   # °/s, °/s²
        self.tcp_velocity, self.tcp_acceleration, self.tcp_angular = tcp_velocity, tcp_acceleration, tcp_angular

    @classmethod
    def from_configuration(cls, values):
        try:
            velocity = [float(values[f'joint.{j}.velocity']) for j in range(1, 7)]
            acceleration = [float(values[f'joint.{j}.acceleration']) for j in range(1, 7)]
        except (KeyError, TypeError, ValueError):
            raise ProgramFailure('Konfiguracja robota: brak limitów osi (joint.N.velocity / joint.N.acceleration)') from None
        tcp = [values.get(key) for key in ('tcp.max_linear_velocity', 'tcp.linear_acceleration', 'tcp.max_angular_velocity')]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in velocity + acceleration):
            raise ProgramFailure('Konfiguracja robota: limity osi muszą być dodatnie')
        return cls(velocity, acceleration, *[float(v) if type(v) in (int, float) else None for v in tcp])

    def segment(self, move):
        """Ruch → pola MotionSegment (SI). PTP: % najwolniejszej osi (jak dawny Mover); LIN: mm/s ≤ tcp.max_linear_velocity."""
        if not 0 <= move.blend_mm <= BLEND_MAX_MM:
            raise ProgramFailure(f'Promień pominięcia {move.blend_mm:g} mm poza zakresem 0–{BLEND_MAX_MM:g} mm', move.block_id)
        segment = {'tag': move.block_id, 'motion_type': move.kind, 'target_positions': [math.radians(q) for q in move.joints_deg],
                   'blend_radius': move.blend_mm / 1000, 'tcp_velocity': 0., 'tcp_acceleration': 0., 'tcp_angular_velocity': 0.}
        if move.kind == 'PTP':
            if not 0 < move.speed <= 100:
                raise ProgramFailure(f'PTP: prędkość {move.speed:g}% poza zakresem 1–100%', move.block_id)
            v, a = min(self.joint_velocity) * move.speed / 100, min(self.joint_acceleration) * move.speed / 100
            segment['max_joint_velocity'] = [math.radians(min(v, limit)) for limit in self.joint_velocity]
            segment['max_joint_acceleration'] = [math.radians(min(a, limit)) for limit in self.joint_acceleration]
            return segment
        if None in (self.tcp_velocity, self.tcp_acceleration, self.tcp_angular):
            raise ProgramFailure('Konfiguracja robota: brak limitów TCP (tcp.max_linear_velocity / linear_acceleration / '
                                 'max_angular_velocity)', move.block_id)
        if not 0 < move.speed <= self.tcp_velocity:
            raise ProgramFailure(f'LIN: prędkość {move.speed:g} mm/s poza zakresem (0, {self.tcp_velocity:g}] z konfiguracji', move.block_id)
        segment['max_joint_velocity'] = [math.radians(v) for v in self.joint_velocity]
        segment['max_joint_acceleration'] = [math.radians(a) for a in self.joint_acceleration]
        segment.update(tcp_velocity=move.speed / 1000, tcp_acceleration=max(100., self.tcp_acceleration) / 1000,
                       tcp_angular_velocity=math.radians(min(360., max(10., self.tcp_angular))))
        return segment

    def estimate(self, start_deg, move):
        """Przybliżony czas ruchu [s] (podział na ciągi; dokładny czas liczy planer)."""
        if start_deg is None:
            return 0.
        if move.kind == 'PTP':
            v, a = min(self.joint_velocity) * move.speed / 100, min(self.joint_acceleration) * move.speed / 100
        else:
            v, a = min(self.joint_velocity), min(self.joint_acceleration)
        return max(abs(q - p) for q, p in zip(move.joints_deg, start_deg)) / v + v / a


class ChainBuilder:
    """Pobiera zdarzenia interpretera z podglądem jednego naprzód i składa ciągi ruchów."""

    def __init__(self, events, limits):
        self.events, self.limits = iter(events), limits
        self.pending = deque()
        self.finished = False

    def peek(self):
        if not self.pending and not self.finished:
            try:
                self.pending.append(next(self.events))
            except StopIteration:
                self.finished = True
        return self.pending[0] if self.pending else None

    def take(self):
        event = self.peek()
        if event is not None:
            self.pending.popleft()
        return event

    def push_front(self, moves):
        """Ruchy nieujęte przez planer (ucięcie na postoju) wracają na początek kolejki."""
        for move in reversed(moves):
            self.pending.appendleft(move)

    def more(self):
        return self.peek() is not None

    def next_chain(self, start_deg=None):
        """→ (ruchy, ostrzeżenia). Ruchy = kopie zdarzeń Move (ostatni z blend_mm=0)."""
        moves, warnings, total, position = [], [], 0., start_deg
        while True:
            event = self.peek()
            if event is None:
                break
            if not isinstance(event, Move):   # Sync: koniec ciągu (zdarzenie obsługuje wykonawca)
                if not moves:
                    self.take()
                    warnings.append({'block_id': event.block_id, 'code': 'sync', 'text': event.reason})
                    continue
                break
            if len(moves) >= MAX_CHAIN_SEGMENTS or (moves and total >= CHAIN_ESTIMATE_S):
                break
            self.take()
            moves.append(Move(**vars(event)))
            total += self.limits.estimate(position, event)
            position = event.joints_deg
        if moves and moves[-1].blend_mm > 0:
            nxt = self.peek()
            if nxt is None:
                code, text = 'chain_end', 'Ostatni ruch programu — postój w punkcie'
            elif isinstance(nxt, Move):
                code, text = 'chain_split', 'Podział programu na ciągi (limit czasu/liczby ruchów) — postój w punkcie'
            else:
                code, text = 'blend_not_possible', 'Następny ruch nieznany (punkt synchronizacji) — postój w punkcie'
            warnings.append({'block_id': moves[-1].block_id, 'code': code, 'text': text})
            moves[-1].blend_mm = 0.
        return moves, warnings


def robot_cursor(timings, elapsed, state='running'):
    """Blok wykonywany przez robota w chwili `elapsed` [s od startu ciągu] wg czasów planera (chain_segments)."""
    if not timings:
        return None
    if state in ('arming', 'ready', 'preparing'):
        return {'block_id': timings[0]['tag'], 'phase': 'arming', 'blend': None}
    if state in ('settling', 'waiting_position', 'waiting_feedback', 'restoring', 'restored', 'succeeded'):
        return {'block_id': timings[-1]['tag'], 'phase': 'settling' if state != 'succeeded' else 'done', 'blend': None}
    t = max(0., float(elapsed or 0.))
    for k, timing in enumerate(timings):
        last = k + 1 == len(timings)
        if t >= timing['transition_end'] and not last:
            continue
        if t < timing['end_time']:
            return {'block_id': timing['tag'], 'phase': 'moving', 'blend': None}
        if timing.get('stop', True) or last:
            return {'block_id': timing['tag'], 'phase': 'stopped', 'blend': None}
        nxt = timings[k + 1]['tag']
        progress = min(1., max(0., (t - timing['end_time']) / max(1e-9, timing['transition_end'] - timing['end_time'])))
        return {'block_id': timing['tag'] if progress < .5 else nxt, 'phase': 'blending',
                'blend': {'from': timing['tag'], 'to': nxt, 'progress': round(progress, 3)}}
