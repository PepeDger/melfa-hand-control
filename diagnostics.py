"""Read-only R3 telemetry. This is an operational interlock, not a safety system.

Field/bit meanings cross-checked against Robotics Library MitsubishiR3's
doDstate/doState (https://github.com/roboticslibrary/rl). Actual CR2D frames are
fixtures in tests. STATE is slot 1 only; it cannot certify all 32 task slots.
"""
import re


def decode_flags(run, stop, error):
    run, stop = int(run, 16), int(stop, 16)
    return dict(servo_on=bool(run & 32), automatic=not bool(run & 8),
                running=bool(run & 64), operation_enabled=bool(run & 128),
                machine_lock=bool(run & 4), teach_running=bool(run & 16),
                emergency_stop=bool(stop & 1), stopped=bool(stop & 2), waiting=bool(stop & 4),
                pseudo_input=bool(stop & 64), error=int(error),
                run_hex=f'{run:02X}', stop_hex=f'{stop:02X}')


def parse_dstate(payload):
    match = re.fullmatch(r'([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})([0-9]{1,5});([0-9]{1,8});([0-3])', payload)
    if not match:
        raise ValueError('Неизвестный/неполный формат DSTATE: ' + repr(payload))
    run, stop, error, step, mechanism = match.groups()
    return dict(decode_flags(run, stop, error), step=int(step), mechanism=int(mechanism))


def parse_state(payload):
    fields = payload.split(';')
    if len(fields) != 19 or any(any(ord(c) < 32 or ord(c) > 126 for c in f) for f in fields):
        raise ValueError('Неизвестный/неполный формат STATE: ' + repr(payload))
    if not (re.fullmatch(r'[0-9]{1,8}', fields[1]) and
            re.fullmatch(r'(?:100|[0-9]{1,2})', fields[2]) and
            re.fullmatch(r'[0-9A-Fa-f]{1,2}', fields[3]) and
            re.fullmatch(r'[0-9]{1,8}', fields[5]) and
            re.fullmatch(r'[0-9A-Fa-f]{1,2}', fields[6]) and
            fields[15] in ('REP', 'CYC') and fields[16] in ('START', 'ALWAYS', 'ERROR') and
            re.fullmatch(r'[0-9]{1,2}', fields[17]) and re.fullmatch(r'[0-3]', fields[18])):
        raise ValueError('Некорректные поля STATE: ' + repr(payload))
    header = re.fullmatch(r'([0-9A-Fa-f]{2})([0-9A-Fa-f]{2})([0-9]{1,5})', fields[4])
    if not header:
        raise ValueError('Некорректный статус STATE: ' + repr(fields[4]))
    flags = decode_flags(*header.groups())
    edit = int(fields[3], 16)
    return dict(flags, program=fields[0], line=int(fields[1]), override=int(fields[2]),
                editing=bool(edit & 1), program_running=bool(edit & 2),
                task_program=fields[14], task_mode=fields[15], task_condition=fields[16],
                priority=int(fields[17]), mechanism=int(fields[18]))


def status_frame_complete(command, payload):
    try:
        (parse_dstate if command == 'DSTATE' else parse_state)(payload)
        return True
    except ValueError:
        return False


def stopped_wait(status):
    """A stopped interpreter, not proof that every task in the cell is idle."""
    return bool(status and status['waiting'] and status.get('stopped', False) and
                not status['running'] and not status.get('program_running', False) and
                not status.get('editing', False) and not status['teach_running'])


def movement_blockers(status, during_move=False, require_rights=True, operator_session=False, require_servo=True):
    if not status:
        return ['Состояние контроллера ещё не прочитано']
    reasons = []
    for condition, message in (
        (require_servo and not status['servo_on'], 'Серво выключено; включите SVO ON'),
        (not status['automatic'], 'Контроллер в MANUAL'),
        (require_rights and not status['operation_enabled'], 'Контроллер не предоставил право управления'),
        (status['machine_lock'], 'Включён Machine lock'),
        (status['emergency_stop'], 'Активен аварийный останов'),
        (status['error'] != 0, f"Ошибка контроллера {status['error']}"),
        (status['teach_running'], 'Выполняется движение с пульта'),
        (status['pseudo_input'], 'Активен режим имитации входов'),
        (not during_move and status['running'], 'Контроллер занят выполнением'),
        (not during_move and status.get('program_running', False), 'В первом слоте выполняется программа'),
        (not during_move and not status.get('stopped', False), 'Контроллер не подтвердил остановленный контекст'),
        (status.get('editing', False), 'Программа первого слота редактируется'),
        (not during_move and status['waiting'] and not (operator_session and stopped_wait(status)),
         'WAIT: требуется подтверждённый оператором остановленный контекст без RUN'),
    ):
        if condition:
            reasons.append(message)
    return reasons
