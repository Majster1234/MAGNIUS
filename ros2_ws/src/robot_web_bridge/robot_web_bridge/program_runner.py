"""Wykonawca programów ruchowych w moście: dwa kursory, ciągi ruchów liczone z wyprzedzeniem, sprawdzenie i wykonanie.

- Kursor programu: interpreter przechodzi bloki i składa kolejny ciąg ruchów, zanim robot skończy bieżący.
- Kursor robota: blok wykonywany fizycznie (czas wykonawcy względem czasów ruchów planu ciągu).
- Sprawdzenie (check): symulacja całego programu od bieżącej pozycji — oś czasu per blok, bez ruchu.
- Wykonanie (execute): tylko zapisana rewizja zatwierdzona przez operatora w GUI; wykonawca nigdy nie zatwierdza.
- Punkty synchronizacji między ciągami: czekanie (sen po dolocie) i czekanie na operatora (``resume`` z GUI).
Zależności (planer, wykonawca ruchu, magazyn programów) są wstrzykiwane — testy używają atrap.
"""
import asyncio
import math
import time
import uuid
from copy import deepcopy

from .motion_chain import CHAIN_MAX_DURATION_S, ChainBuilder, MotionLimits, joint_ranges, limit_violations, robot_cursor
from .program_interpreter import Interpreter, ProgramFailure

CHAIN_GAP_S = 2.0            # szacowana przerwa między ciągami (dolot, przywrócenie trybu, uzbrojenie)
START_TOLERANCE_DEG = .25    # rozjazd pozycji względem przewidzianego startu ciągu → przeliczenie od pomiaru
FAILED_STATES = ('failed', 'fault', 'invalidated', 'cancelled', 'aborted', 'stopped')


class RunConflict(ValueError):
    pass


class PlannerBusy(RuntimeError):
    pass


def used_positions(document):
    """Pozycje, do których program może pojechać: odwołania {"position"} w blokach włączonych i wartości POZYCJA zmiennych."""
    out = {}

    def walk(blocks):
        for block in blocks:
            if block.get('disabled'):
                continue
            for value in block.get('params', {}).values():
                if isinstance(value, dict) and 'position' in value:
                    out.setdefault(value['position'], block['id'])
            for key, child in block.items():
                if isinstance(child, list):
                    walk(child)
    walk(document.get('blocks', []))
    for variable in document.get('variables', []):
        if variable.get('type') == 'POZYCJA' and variable.get('init'):
            out.setdefault(variable['init'], None)
    return out


def position_limit_report(document, values):
    """Pozycje programu poza zakresem osi → [{name, used, block_id, violations: [{axis, value, lower, upper}]}]."""
    ranges, used, report = joint_ranges(values), used_positions(document), []
    for position in document.get('positions', []):
        bad = limit_violations(position['joints_deg'], ranges)
        if bad:
            report.append({'name': position['name'], 'used': position['name'] in used, 'block_id': used.get(position['name']),
                           'violations': [{'axis': a, 'value': q, 'lower': lo, 'upper': hi} for a, q, lo, hi in bad]})
    return report


def describe_violations(item):
    return ', '.join(f"J{v['axis']} = {v['value']:g}° (zakres {v['lower']:g}…{v['upper']:g}°)" for v in item['violations'])


def iso_now():
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime())


class ProgramRunner:
    def __init__(self, store, planning, motion):
        self.store, self.planning, self.motion = store, planning, motion
        self.task = None
        self.resume_event = None
        self.last_check = None
        self.status = self.idle()

    @staticmethod
    def idle():
        return {'run_id': None, 'mode': None, 'state': 'idle', 'program_id': None, 'revision': None, 'name': None,
                'started_at': None, 'finished_at': None, 'message': '',
                'program_cursor': {'block_id': None, 'state': 'done'}, 'robot_cursor': None,
                'chain': None, 'next_chain': None, 'chains': 0, 'warnings': [], 'error': None, 'wait': None,
                'timeline': [], 'total_s': None}

    def snapshot(self):
        out = deepcopy(self.status)
        out['active'] = self.active()
        out['server_monotonic_s'] = time.monotonic()
        return out

    def active(self):
        return self.task is not None and not self.task.done()

    # ----- API -----
    async def start(self, identifier, mode, revision, client=''):
        if mode not in ('check', 'execute'):
            raise ValueError('Tryb: check albo execute')
        if self.active():
            raise RunConflict('Trwa inny przebieg programu; zatrzymaj go najpierw')
        reply = self.store.get(identifier)
        document, problems = reply['program'], reply['problems']
        if revision is not None and revision != document.get('revision'):
            raise RunConflict('Rewizja programu różni się od zapisanej; zapisz i odśwież program')
        if problems:
            raise ValueError('Program ma uwagi: ' + '; '.join(problems[:5]))
        if mode == 'execute':
            start = await self.planning.program_start()
            blocking = [item for item in position_limit_report(document, start['values']) if item['used']]
            if blocking:
                raise ValueError('Pozycje poza zakresem osi: ' + '; '.join(f"{i['name']}: {describe_violations(i)}" for i in blocking))
        if mode == 'execute' and not self.store.approved(document):
            raise ValueError('Ta rewizja programu NIE jest zatwierdzona przez operatora (GUI → „✔ Zatwierdź do wykonania”)')
        if mode == 'execute':
            status = await self.motion.status()
            if status.get('busy') or getattr(self.motion, 'owner', None):
                raise RunConflict('Trwa inny ruch robota')
        self.begin(mode, document)
        self.task = asyncio.create_task(self.guarded(self.execute(document) if mode == 'execute' else self.check(document)))
        return self.snapshot()

    async def preflight(self, identifier):
        """Analiza przed uruchomieniem (bez ruchu): program, zakresy osi pozycji, stan robota i napędów.

        Kroki: {id, label, state: ok|warn|error|action, detail, items?}. ``action`` = do wykonania automatycznie przed
        startem (np. załączenie napędów). ``ok`` = brak błędów. Symulację ruchów wykonuje osobno sprawdzenie (check).
        """
        reply = self.store.get(identifier)
        document, problems = reply['program'], reply['problems']
        steps = [{'id': 'program', 'label': 'Program zapisany, bez uwag', 'state': 'error' if problems else 'ok',
                  'detail': '; '.join(problems[:5]) if problems else f"rewizja {document.get('revision')}"}]
        start = await self.planning.program_start()
        values, ranges = start['values'], joint_ranges(start['values'])
        report = position_limit_report(document, values)
        used = [item for item in report if item['used']]
        steps.append({'id': 'limits', 'label': 'Pozycje w zakresie osi',
                      'state': 'error' if used else 'warn' if report else 'ok',
                      'detail': '; '.join(f"{i['name']}{'' if i['used'] else ' (nieużywana)'}: {describe_violations(i)}" for i in report)
                      or f"{len(document.get('positions', []))} pozycji w zakresie" + ('' if all(ranges) else ' (część osi bez zakresu w konfiguracji)'),
                      'items': report})
        state = self.planning.robot_state()
        axes, errors = state.get('axes', []), []
        if self.active():
            errors.append('trwa inny przebieg programu')
        status = await self.motion.status()
        if status.get('busy') or getattr(self.motion, 'owner', None):
            errors.append('wykonawca ruchu jest zajęty')
        if not state.get('fresh'):
            errors.append('brak świeżego stanu osi')
        if state.get('fault'):
            errors.append(f"awaria: {state.get('message') or 'stan awaryjny'}")
        if state.get('recovery_state', 'idle') not in ('idle', 'idle_disabled'):
            errors.append(f"odzyskiwanie po awarii w toku: {state.get('recovery_message') or state.get('recovery_state')}")
        if (state.get('tcp_jog') or {}).get('busy') or state.get('busy'):
            errors.append('trwa ruch ręczny (JOG)')
        for k in range(6):
            axis = axes[k] if k < len(axes) else {}
            label = f'J{k + 1}'
            if not axis.get('available'):
                errors.append(f'{label}: napęd niedostępny')
            elif not axis.get('joint_configured') or not axis.get('calibration_valid'):
                errors.append(f'{label}: brak kalibracji/profilu przegubu')
            elif not axis.get('joint_position_known'):
                errors.append(f'{label}: nieznana pozycja')
            elif ranges[k] and not ranges[k][0] - 1e-6 <= axis['joint_position_degrees'] <= ranges[k][1] + 1e-6:
                errors.append(f"{label}: bieżąca pozycja {axis['joint_position_degrees']:.2f}° poza zakresem {ranges[k][0]:g}…{ranges[k][1]:g}°")
        steps.append({'id': 'robot', 'label': 'Stan robota', 'state': 'error' if errors else 'ok',
                      'detail': '; '.join(errors) if errors else 'pozycje zmierzone, kalibracja, bez awarii, wykonawca wolny',
                      'current_degrees': [a.get('joint_position_degrees') for a in axes[:6]]})
        disabled = [k + 1 for k in range(6) if k < len(axes) and not (axes[k].get('enable_known') and axes[k].get('enabled'))]
        steps.append({'id': 'drives', 'label': 'Napędy załączone (ENABLE)', 'state': 'action' if disabled else 'ok',
                      'detail': f"do załączenia przed startem: {', '.join(f'J{a}' for a in disabled)}" if disabled else 'J1–J6 załączone',
                      'disabled': disabled})
        return {'program_id': identifier, 'revision': document.get('revision'), 'approved': self.store.approved(document),
                'ok': not any(step['state'] == 'error' for step in steps), 'steps': steps}

    async def check_document(self, document):
        """Sprawdzenie niezapisanego programu (np. przygotowanego przez agenta) — bez wykonania."""
        if self.active():
            raise RunConflict('Trwa inny przebieg programu')
        checked = self.store.check(document)
        if checked['problems']:
            raise ValueError('Program ma uwagi: ' + '; '.join(checked['problems'][:5]))
        document = self.store.validate(document)
        document['revision'] = checked['revision']
        self.begin('check', document)
        self.task = asyncio.create_task(self.guarded(self.check(document, remember=False)))
        return self.snapshot()

    async def resume(self):
        """Kontynuacja po bloku „czekaj na operatora” (przycisk operatora w GUI)."""
        if not self.active() or self.status.get('state') != 'waiting_operator' or self.resume_event is None:
            raise RunConflict('Program nie czeka na operatora')
        self.resume_event.set()
        return self.snapshot()

    async def stop(self):
        stopped = False
        if self.active():
            self.status['state'] = 'stopping'
            self.task.cancel()
            try:
                await asyncio.wait_for(asyncio.shield(self.task), 5)
            except (asyncio.CancelledError, asyncio.TimeoutError, Exception):
                pass
            stopped = True
        if not stopped and self.status.get('mode') == 'execute' and self.status.get('state') not in ('succeeded', 'failed', 'stopped'):
            await self.motion.stop()
        return self.snapshot()

    # ----- przebieg -----
    def begin(self, mode, document):
        self.status = self.idle()
        self.status.update(run_id=uuid.uuid4().hex, mode=mode, state='checking' if mode == 'check' else 'starting',
                           program_id=document.get('id'), revision=document.get('revision'), name=document.get('name'),
                           started_at=iso_now())

    def warn(self, block_id, code, text):
        entry = {'block_id': block_id, 'code': code, 'text': text}
        if entry not in self.status['warnings']:
            self.status['warnings'].append(entry)

    async def guarded(self, work):
        try:
            await work
            self.status.update(state='succeeded', finished_at=iso_now())
            self.status['program_cursor'] = {'block_id': None, 'state': 'done'}
        except asyncio.CancelledError:
            if self.status['mode'] == 'execute':
                await self.safe_stop()
            self.status.update(state='stopped', finished_at=iso_now(), message='Przebieg zatrzymany przez operatora')
            raise
        except Exception as error:
            if self.status['mode'] == 'execute':
                await self.safe_stop()
            block = getattr(error, 'block_id', None)
            self.status.update(state='failed', finished_at=iso_now(), message=str(error),
                               error={'block_id': block, 'text': str(error) or type(error).__name__})
        finally:
            if self.status['mode'] == 'execute' and hasattr(self.motion, 'release'):
                self.motion.release(self.status['run_id'])

    async def safe_stop(self):
        try:
            await self.motion.stop()
        except Exception:
            pass

    def cursor(self, block_id, state):
        self.status['program_cursor'] = {'block_id': block_id, 'state': state}

    async def plan(self, builder, moves, seeds, start, physical, limits):
        """Plan ciągu; ucięcie przez planer → reszta wraca do kolejki; zbyt długi odcinek → wymuszony postój."""
        for attempt in range(8):
            segments = [limits.segment(move) for move in moves]
            context = await self.planning.chain_context(seeds, physical, start)
            def feedback(progress, phase):
                self.status['next_chain'] = {**(self.status.get('next_chain') or {}), 'progress': round(float(progress), 3), 'phase': phase}
            result = None
            for retry in range(4):
                try:
                    result = await self.planning.plan_chain(context, segments, CHAIN_MAX_DURATION_S, feedback)
                    break
                except PlannerBusy:
                    if retry == 3:
                        raise
                    await asyncio.sleep(.5)
            if result['success']:
                planned = result['planned_segments']
                if planned < len(moves):
                    builder.push_front(moves[planned:])
                    moves = moves[:planned]
                for raw in result['warnings']:
                    index, code, text = (raw.split('|', 2) + ['', ''])[:3]
                    block = moves[int(index)].block_id if index.isdigit() and int(index) < len(moves) else None
                    self.warn(block, code, text)
                return {'moves': moves, **result}
            failed = result.get('failed_segment', -1)
            if 'limit czasu ciągu' in result['message'] and len(moves) > 1:
                blended = [k for k, move in enumerate(moves[:-1]) if move.blend_mm > 0]
                if blended:
                    k = blended[len(blended) // 2]
                    moves[k].blend_mm = 0.
                    self.warn(moves[k].block_id, 'chain_split', 'Podział długiego ciągu przejść — postój w punkcie')
                    continue
            block = moves[failed].block_id if 0 <= failed < len(moves) else (moves[0].block_id if moves else None)
            raise ProgramFailure(result['message'] or 'Planer odrzucił ciąg ruchów', block)
        raise ProgramFailure('Nie udało się podzielić ciągu ruchów', moves[0].block_id if moves else None)

    async def check(self, document, remember=True):
        start = await self.planning.program_start()
        seeds, sources = list(start['seeds']), start['sources']
        if any(source == 'zero' for source in sources):
            self.warn(None, 'start_assumed', 'Nieznana pozycja części osi — sprawdzenie od pozycji 0°')
        limits = MotionLimits.from_configuration(start['values'])
        interpreter = Interpreter(document)
        builder = ChainBuilder(interpreter.events(), limits)
        start['simulated_start'] = [source != 'measured' for source in sources]
        timeline, offset, index = [], 0., 0
        while builder.more():
            self.cursor(interpreter.cursor, 'computing')
            moves, warnings = builder.next_chain(seeds)
            for item in warnings:
                self.warn(item['block_id'], item['code'], item['text'])
            for event in builder.leading:
                duration = event.seconds if event.kind == 'wait' else 0.
                timeline.append({'block_id': event.block_id, 'kind': event.kind, 'chain': index, 'start_s': round(offset, 3),
                                 'end_s': round(offset + duration, 3), 'transition_end_s': round(offset + duration, 3), 'stop': True,
                                 'blend_mm_requested': 0., 'blend_mm_applied': 0., 'text': event.reason})
                offset += duration
                if event.kind == 'operator':
                    self.warn(event.block_id, 'operator', f'Czekanie na operatora: {event.reason} (czas nieznany)')
            if not moves:
                continue
            if index > 0:
                offset += CHAIN_GAP_S
            await self.resolve(moves, seeds, start)
            planned = await self.plan(builder, moves, seeds, start, False, limits)
            for timing in planned['timings']:
                timeline.append({'block_id': timing['tag'], 'chain': index, 'start_s': round(offset + timing['start_time'], 3),
                                 'end_s': round(offset + timing['end_time'], 3), 'transition_end_s': round(offset + timing['transition_end'], 3),
                                 'stop': timing['stop'], 'blend_mm_requested': round(timing['requested_blend_radius'] * 1000, 3),
                                 'blend_mm_applied': round(timing['applied_blend_radius'] * 1000, 3)})
            offset += planned['duration']
            seeds = planned['target_degrees']
            index += 1
            self.status.update(timeline=list(timeline), chains=index, total_s=round(offset, 2))
        self.status.update(timeline=timeline, chains=index, total_s=round(offset, 2),
                           message=f'Sprawdzenie OK: {sum(1 for e in timeline if "kind" not in e)} ruchów, {index} ciąg(i), ~{offset:.1f} s')
        if remember:
            self.last_check = {'key': (document.get('id'), document.get('revision'), start['configuration_revision']),
                               'seeds': list(start['seeds']), 'timeline': timeline, 'total_s': round(offset, 2), 'chains': index}
        return timeline

    def check_matches(self, document, start):
        last = self.last_check
        return (last is not None and last['key'] == (document.get('id'), document.get('revision'), start['configuration_revision'])
                and max(abs(a - b) for a, b in zip(last['seeds'], start['seeds'])) <= .5)

    async def execute(self, document):
        token = self.status['run_id']
        start = await self.planning.program_start()
        if not self.check_matches(document, start):
            self.status['state'] = 'checking'
            await self.check(document)
        else:
            self.status.update(timeline=self.last_check['timeline'], total_s=self.last_check['total_s'], chains=self.last_check['chains'])
        check_summary = {key: self.status[key] for key in ('timeline', 'total_s', 'chains')}
        self.motion.claim(token)
        self.status.update(state='planning', **check_summary)
        start = await self.planning.program_start()
        limits = MotionLimits.from_configuration(start['values'])
        interpreter = Interpreter(document)
        builder = ChainBuilder(interpreter.events(), limits)
        seeds = self.motion.measured_degrees()

        async def compute(from_deg):
            self.cursor(interpreter.cursor, 'computing')
            leading = []
            while builder.more():
                moves, warnings = builder.next_chain(from_deg)
                leading += builder.leading
                for item in warnings:
                    self.warn(item['block_id'], item['code'], item['text'])
                if moves:
                    await self.resolve(moves, from_deg, start)
                    planned = await self.plan(builder, moves, from_deg, start, True, limits)
                    planned['leading'] = leading
                    self.cursor(planned['moves'][-1].block_id, 'waiting_robot')
                    return planned
            self.cursor(None, 'done')
            return {'moves': [], 'leading': leading} if leading else None

        planned = await compute(seeds)
        index = 0
        while planned is not None:
            if planned['leading']:
                await self.hold(planned['leading'])
                self.verify(document)
            if not planned['moves']:
                break
            if index > 0:
                measured = self.motion.measured_degrees()
                if max(abs(a - b) for a, b in zip(measured, planned['seeds_deg'])) > START_TOLERANCE_DEG:
                    self.warn(planned['moves'][0].block_id, 'replanned', 'Pozycja po ciągu różni się od planu — przeliczono od pomiaru')
                    planned = await self.plan(builder, planned['moves'], measured, start, True, limits)
            self.status.update(state='preparing', chain={'index': index, 'duration_s': planned['duration'], 'elapsed_s': 0.,
                               'executor_state': 'preparing', 'segments': planned['timings']}, next_chain=None)
            self.status['robot_cursor'] = robot_cursor(planned['timings'], 0, 'preparing')
            program_id = await self.motion.prepare_chain(planned['plan'], token)
            await self.motion.run_execute(program_id, f'program-{token[:12]}-{index}', token)
            follow = None
            if builder.more():
                self.status['next_chain'] = {'index': index + 1, 'state': 'planning'}
                follow = asyncio.create_task(compute(planned['target_degrees']))
            try:
                await self.watch(program_id, planned)
                upcoming = await follow if follow else None
            except BaseException:
                if follow:
                    follow.cancel()
                    await asyncio.gather(follow, return_exceptions=True)
                raise
            # Granica ciągu: program nie mógł się zmienić ani stracić zatwierdzenia w trakcie.
            self.verify(document)
            if upcoming is not None:
                upcoming['seeds_deg'] = planned['target_degrees']   # przewidziany start (kontrola po ewentualnym czekaniu)
                self.status['next_chain'] = None
            planned = upcoming
            index += 1
            self.status['chains'] = max(self.status['chains'] or 0, index)
        self.status['robot_cursor'] = None
        self.status['message'] = f'Program wykonany ({index} ciąg(i))'

    async def resolve(self, moves, start_deg, start):
        """Cele ruchów względnych: FK celu poprzedniego ruchu (lub startu ciągu) → przyrost TCP → IK z ziarnem poprzedniego celu."""
        previous = list(start_deg)
        for move in moves:
            if move.joints_deg is None:
                try:
                    move.joints_deg = list(await self.planning.relative_target(previous, start, move.relative))
                except (ValueError, RuntimeError) as error:
                    raise ProgramFailure(f'Ruch względny: {error}', move.block_id) from None
            previous = list(move.joints_deg)

    def verify(self, document):
        current = self.store.get(document['id'])['program']
        if current.get('revision') != document.get('revision') or not self.store.approved(current):
            raise ProgramFailure('Program zmieniony lub cofnięto zatwierdzenie — zatrzymano na granicy ciągu')

    async def hold(self, events):
        """Punkty synchronizacji przed ciągiem: czekanie (robot stoi po dolocie) albo czekanie na „Kontynuuj” operatora."""
        for event in events:
            self.status.update(chain=None, next_chain=None)
            self.cursor(event.block_id, 'waiting_sync')
            if event.kind == 'wait':
                self.status.update(state='waiting', message=event.reason,
                                   wait={'block_id': event.block_id, 'kind': 'wait', 'seconds': event.seconds,
                                         'until_monotonic_s': time.monotonic() + event.seconds, 'text': event.reason})
                self.status['robot_cursor'] = {'block_id': event.block_id, 'phase': 'waiting', 'blend': None}
                await asyncio.sleep(event.seconds)
            elif event.kind == 'operator':
                self.resume_event = asyncio.Event()
                self.status.update(state='waiting_operator', message=event.reason,
                                   wait={'block_id': event.block_id, 'kind': 'operator', 'text': event.reason})
                self.status['robot_cursor'] = {'block_id': event.block_id, 'phase': 'operator', 'blend': None}
                try:
                    await self.resume_event.wait()
                finally:
                    self.resume_event = None
            self.status.update(wait=None, message='')

    async def watch(self, program_id, planned):
        deadline = time.monotonic() + 2 * planned['duration'] + 30
        while True:
            status = await self.motion.status()
            if status.get('program_id') != program_id:
                raise ProgramFailure('Wykonawca ruchu prowadzi inny program')
            state = status.get('state')
            chain = self.status.get('chain') or {}
            chain.update(elapsed_s=round(float(status.get('elapsed') or 0.), 3), executor_state=state)
            self.status['chain'] = chain
            self.status['robot_cursor'] = robot_cursor(planned['timings'], status.get('elapsed'), state)
            if state in ('arming', 'running', 'settling', 'waiting_position', 'waiting_feedback', 'restoring', 'restored'):
                self.status['state'] = 'running' if state == 'running' else ('arming' if state == 'arming' else 'settling')
            if state == 'succeeded' and not status.get('busy'):
                return
            if state in FAILED_STATES:
                raise ProgramFailure(f'Wykonawca: {state} — {status.get("message", "")}',
                                     (self.status['robot_cursor'] or {}).get('block_id'))
            if time.monotonic() > deadline:
                raise ProgramFailure('Ciąg przekroczył limit czasu wykonania', (self.status['robot_cursor'] or {}).get('block_id'))
            await asyncio.sleep(.1)


def add_program_run_routes(app, runner):
    import json
    from fastapi import HTTPException, Request, WebSocket
    from starlette.responses import JSONResponse

    async def body(request):
        raw = bytearray()
        async for chunk in request.stream():
            raw.extend(chunk)
            if len(raw) > 1024 * 1024 + 4096:
                raise HTTPException(413, 'Treść przekracza 1 MiB')
        try:
            return json.loads(raw or b'{}')
        except ValueError:
            raise HTTPException(422, 'Nieprawidłowy JSON') from None

    async def call(operation, *args, status=200):
        if runner is None:
            raise HTTPException(503, 'Wykonawca programów jest niedostępny')
        from .programs import ProgramConflict, ProgramError
        from .motion import MotionConflict
        try:
            return JSONResponse(await operation(*args), status_code=status, headers={'Cache-Control': 'no-store'})
        except (RunConflict, ProgramConflict, MotionConflict) as error:
            raise HTTPException(409, str(error)) from None
        except KeyError as error:
            raise HTTPException(404, str(error.args[0] if error.args else 'Nie znaleziono')) from None
        except (ValueError, ProgramError, ProgramFailure) as error:
            raise HTTPException(422, str(error)) from None
        except (RuntimeError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from None

    @app.post('/api/v1/programs/{identifier}/run')
    async def run(identifier: str, request: Request):
        data = await body(request)
        client = request.client.host if request.client else ''
        return await call(runner.start if runner else None, identifier, data.get('mode'), data.get('revision'), client, status=202)

    @app.post('/api/v1/programs/{identifier}/preflight')
    async def preflight(identifier: str):
        return await call(runner.preflight if runner else None, identifier)

    @app.post('/api/v1/program-run/check-document')
    async def check_document(request: Request):
        data = await body(request)
        return await call(runner.check_document if runner else None, data.get('program', data), status=202)

    @app.get('/api/v1/program-run')
    async def status():
        async def snapshot():
            return runner.snapshot()
        return await call(snapshot if runner else None)

    @app.post('/api/v1/program-run/resume')
    async def resume():
        return await call(runner.resume if runner else None)

    @app.post('/api/v1/program-run/stop')
    async def stop():
        return await call(runner.stop if runner else None)

    @app.websocket('/api/v1/program-run/ws')
    async def stream(websocket: WebSocket):
        if runner is None:
            await websocket.close(code=1013)
            return
        await app.state.stream_state(websocket, runner.snapshot, 0.1, 'program-run')
