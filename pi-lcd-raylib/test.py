#!/usr/bin/env python3
from dataclasses import dataclass
from collections import namedtuple
from pyray import *
from joystick import *


def main():
    init_window(800, 450, "raylib [3D] example - basic 3D shapes")
    set_target_fps(60)

    camera = Camera3D(
        Vector3(0, 10, 10),  # Camera position
        Vector3(0, 0, 0),    # Camera target
        Vector3(0, 1, 0),    # Camera up vector
        45.0,                # Camera field-of-view Y
        CAMERA_PERSPECTIVE   # Camera projection type
    )

    # background_color = RAYWHITE
    background_color = BLACK
    
    app_run = True

    gamepad_num = 0
    axes = get_gamepad_axis_count(gamepad_num)
    name = get_gamepad_name(gamepad_num)
    print(f"GamePad {name} Axes: {axes} {is_gamepad_available(gamepad_num)}")

    # disable_cursor() # cursor disappeared, only this window works

    while (not window_should_close()) and app_run:
        # this is NOT perfect, but eventually works
        if is_cursor_on_screen():
            hide_cursor()

        if is_gamepad_available(gamepad_num):
            gpad = get_gamepad(gamepad_num)
            # print(gpad)

            # update_camera(camera, CAMERA_THIRD_PERSON) # CAMERA_FIRST_PERSON)
            x,y = gpad.stick_left
            update_camera_pro(camera, (-y,x,0),(0,0,0), 0)

            if gpad.button == GamepadButton.GAMEPAD_BUTTON_MIDDLE_RIGHT:
                app_run = False
                break
        else:
            update_camera_pro(camera, (0,0,0),(0,0,0), 0)

        begin_drawing()
        clear_background(background_color)


        begin_mode_3d(camera)

        draw_cube(Vector3(0, 0, 0), 2.0, 2.0, 2.0, RED)
        draw_sphere(Vector3(-4, 0, 0), 1.0, BLUE)
        draw_grid(20, 1.0)

        end_mode_3d()

        draw_fps(10, 10)
        end_drawing()

    close_window()

if __name__ == '__main__':
    main()