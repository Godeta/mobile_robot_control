#!/usr/bin/env python3

import time
import math
from ev3dev2.sensor import Sensor, INPUT_1
from ev3dev2.port import LegoPort
from ev3dev2.display import Display

# Set up Pixy camera on INPUT_1
in1 = LegoPort(INPUT_1)
in1.mode = 'auto'
time.sleep(2)

pixy = Sensor(INPUT_1)
pixy.mode = 'SIG1'

# Display
lcd = Display()


def draw_pixy_rect():
    x = pixy.value(1)
    y = pixy.value(2)
    w = pixy.value(3)
    h = pixy.value(4)
    x *= 0.7
    y *= 0.6
    w *= 0.7
    h *= 0.6
    dx = int(w / 2)
    dy = int(h / 2)
    xa = x - dx
    ya = y + dy
    xb = x + dx
    yb = y - dy
    lcd.clear()
    lcd.draw.rectangle((xa, ya, xb, yb), fill='black')
    lcd.update()

# MAIN
if __name__ == "__main__":
    # After goal reached, keep displaying Pixy object
    while True:
        draw_pixy_rect()
        time.sleep(0.1)
