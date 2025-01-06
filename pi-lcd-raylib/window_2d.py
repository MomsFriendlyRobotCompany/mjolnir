#!/usr/bin/env python3
from pyray import *
from enum import IntEnum

class Ops(IntEnum):
    POS = 2
    NEG = -2

class MyRectangle:
    def __init__(self,w,h):
        self.x = 0
        self.y = 0
        self.xe = Ops.POS
        self.ye = Ops.POS
        self.angle = 0

        self.rec = Rectangle(0,0,w,h)

    def update(self):
        if self.x > 800: self.xe = Ops.NEG
        elif self.x < 0: self.xe = Ops.POS

        if self.y > 600: self.ye = Ops.NEG
        elif self.y < 0: self.ye = Ops.POS

        self.x += self.xe
        self.y += self.ye

        self.angle += 0.1

        # draw_rectangle(self.x, self.y, 200, 100, RED)
        # draw_rectangle_rec(rec, RED)
        # draw_rectangle_pro(self.rec, (self.x,self.y), self.angle, RED)
        rec = Rectangle(self.x,self.y,200,100)
        draw_rectangle_pro(rec, (0,0), self.angle, RED)

# Initialize the window
init_window(800, 600, "My 2D Window")

# Set the target frames per second
set_target_fps(60)

# x,y = 0,0
# xe = Ops.POS
# ye = Ops.POS

# rec = Rectangle(x,y,200,100)
rec = MyRectangle(200,100)
print(dir(rec))

# Game loop
while not window_should_close():
    begin_drawing()
    clear_background(RAYWHITE)

    # draw_rectangle(x, y, 200, 100, RED)
    # draw_rectangle_rec(rec, RED)
    rec.update()

    end_drawing()

close_window()