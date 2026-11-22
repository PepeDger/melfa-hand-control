"""Shared hand-input ranges; independent of physical controller override."""
import math

MIN_SENSITIVITY = .5
MAX_SENSITIVITY = 50.


def validate_sensitivity(value):
    if type(value) not in (int, float) or not math.isfinite(value) or not MIN_SENSITIVITY <= value <= MAX_SENSITIVITY:
        raise ValueError('Чувствительность должна быть числом от 0,5 до 50')
    return float(value)


def validate_hand_step(value):
    if type(value) not in (int,float) or not math.isfinite(value) or not .1 <= value <= 5:
        raise ValueError('Шаг кисти должен быть числом от 0,1 до 5°')
    return float(value)
