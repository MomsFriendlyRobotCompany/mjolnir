from dataclasses import dataclass
from collections import namedtuple
from pyray import *


class GPAxis:
    x: float
    y: float

    def __init__(self, x, y):
        self.x = x if abs(x) > 0.001 else 0.0
        self.y = y if abs(y) > 0.001 else 0.0

    def __str__(self):
        return f"GPAxis(x={self.x:.3f}, y={self.y:.3f})"

    def __repr__(self):
        return str(self)

    def __iter__(self):
        """
        Allows: x,y = gp_axis
        """
        for a in [self.x, self.y]:
            yield a

@dataclass
class GamePad:
    stick_left: GPAxis
    stick_right: GPAxis
    trigger_left: float
    trigger_right: float
    button: int


def get_gamepad(gamepad_num):
    a = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_LEFT_X)
    b = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_LEFT_Y)
    c = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_RIGHT_X)
    d = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_RIGHT_Y)
    lt = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_LEFT_TRIGGER)
    rt = get_gamepad_axis_movement(gamepad_num,GamepadAxis.GAMEPAD_AXIS_RIGHT_TRIGGER)
    button = get_gamepad_button_pressed()
    # print("GamePad -----------------------------------------")
    # print(f" Axis: {a:5.1f},{b:5.1f} {c:5.1f},{d:5.1f}")
    # print(f" Button: {button}")
    # print(f" Trigger: {lt:5.1f} {rt:5.1f}")
    
    # shift offset, 0: unpressed, 1: pressed
    lt = (lt + 1.0) * 0.5
    rt = (rt + 1.0) * 0.5

    gpad = GamePad(GPAxis(a,b),GPAxis(c,d),lt,rt,button)
    return gpad