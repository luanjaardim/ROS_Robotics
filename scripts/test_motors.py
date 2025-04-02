#!/usr/bin/env python3
import os

# twist:
#   linear: {x: 0.0, y: 0.0, z: 0.0}
#  angular: {x: 0.0, y: 0.0, z: 1.0}"

rostopic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 100.0}}" -r 5


while True:
    keyboard_input = input()
    for char in keyboard_input:
        if char == 'w':
            pass
        elif char == 's':
            pass
        elif char == 'a':
            pass
        elif char == 'd':
            pass