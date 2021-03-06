#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFactory, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))
from uarm.wrapper import SwiftAPI
#refer to http://download.ufactory.cc/docs/en/uArm-Swift-Pro-Develper-Guide-171013.pdf, page 4, left/rigt from ego of the arm
from uarm.swift.protocol import SERVO_BOTTOM, SERVO_LEFT, SERVO_RIGHT, SERVO_HAND

"""
api test: attach and detach
"""

swift = SwiftAPI(filters={'hwid': 'USB VID:PID=2341:0042'})

swift.waiting_ready()

print(swift.set_servo_detach(servo_id=SERVO_BOTTOM))
time.sleep(2)
print(swift.set_servo_detach(servo_id=SERVO_LEFT))
time.sleep(2)
print(swift.set_servo_detach(servo_id=SERVO_RIGHT))
time.sleep(2)
print(swift.set_servo_detach(servo_id=SERVO_HAND))

time.sleep(2)
print(swift.set_servo_attach(servo_id=SERVO_BOTTOM))
time.sleep(2)
print(swift.set_servo_attach(servo_id=SERVO_LEFT))
time.sleep(2)
print(swift.set_servo_attach(servo_id=SERVO_RIGHT))
time.sleep(2)
print(swift.set_servo_attach(servo_id=SERVO_HAND))

#deatch all four
print(swift.set_servo_detach())
time.sleep(5)
#attach all four
print(swift.set_servo_attach())
time.sleep(2)

swift.disconnect()
