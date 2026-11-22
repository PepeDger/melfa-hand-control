"""MELFA HNDSTS: output commands for eight pneumatic hands, not jaw sensors."""
import re


def parse_hands(payload):
    fields=payload.split(';')
    if fields and fields[-1]=='': fields.pop()
    if len(fields)!=24 or any(not re.fullmatch(r'-1|[0-9]{1,8}',x) for x in fields):
        raise ValueError('Неизвестный/неполный формат HNDSTS')
    result=[]
    for hand in range(8):
        output,state,kind=map(int,fields[hand*3:hand*3+3])
        if output==-1:
            if (state,kind)!=(-1,-1): raise ValueError('Некорректный неиспользуемый канал HNDSTS')
        elif state not in (1,2) or kind not in (0,1):
            raise ValueError('Неизвестное состояние или тип канала HNDSTS')
        result.append(dict(hand=hand+1,output=output,output_state=state,
                           solenoid_type=kind,closed=None if output==-1 else state==2))
    return result


def hands_frame_complete(payload):
    try:
        parse_hands(payload)
        return True
    except ValueError:
        return False
