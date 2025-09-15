# -*- coding:utf-8 -*-
'''!
  @file DC_Motor_Demo.py
  @brief Connect board with raspberryPi.
  @n Make board power and motor connection correct.
  @n Run this demo.
  @n Motor 1 will move slow to fast, orientation clockwise, 
  @n motor 2 will move fast to slow, orientation count-clockwise, 
  @n then fast to stop. loop in few seconds.
  @n Motor speed will print on terminal
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license    The MIT License (MIT)
  @author     [tangjie](jie.tang@dfrobot.com)
  @version    V1.0.1
  @date       2022-04-19
  @url  https://github.com/DFRobot/DFRobot_RaspberryPi_Motor
'''
from __future__ import print_function
import sys
import os
sys.path.append("../")

import time

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

if THIS_BOARD_TYPE:
  board = Board(1, 0x10)    # RaspberryPi select bus 1, set address to 0x10
else:
  board = Board(7, 0x10)    # RockPi select bus 7, set address to 0x10

def board_detect():
  l = board.detecte()
  print("Board list conform:")
  print(l)

''' print last operate status, users can use this variable to determine the result of a function call. '''
def print_board_status():
  if board.last_operate_status == board.STA_OK:
    print("board status: everything ok")
  elif board.last_operate_status == board.STA_ERR:
    print("board status: unexpected error")
  elif board.last_operate_status == board.STA_ERR_DEVICE_NOT_DETECTED:
    print("board status: device not detected")
  elif board.last_operate_status == board.STA_ERR_PARAMETER:
    print("board status: parameter error, last operate no effective")
  elif board.last_operate_status == board.STA_ERR_SOFT_VERSION:
    print("board status: unsupport board framware version")

# physical measurements in mm
WHEEL_DIAMETER_MM = 30
TRACK_WIDTH_MM = 170

import math, time

def drive_distance(board, distance_m, speed_pct, wheel_diameter_m=WHEEL_DIAMETER_MM, gear_ratio=50,
                   slow_down_window_m=0.10, min_speed_pct=15.0, poll_dt=0.01,
                   watchdog_s=1.5):
    """
    Drive straight for distance_m (meters) at speed_pct (%) and stop.
    Positive distance_m => CW, negative => CCW.
    Uses get_encoder_speed() [RPM] and integrates to distance.
    wheel_diameter_m: physical wheel diameter in meters.
    gear_ratio: set to your gearbox ratio so the HAT reports *wheel* RPM (e.g., 50).

    Returns the measured distance traveled (meters).
    """

    # 1) Ensure encoders are enabled and the ratio is correct so RPM == wheel RPM
    board.set_encoder_enable([board.M1, board.M2])
    board.set_encoder_reduction_ratio([board.M1, board.M2], int(gear_ratio))

    # 2) Decide direction and start moving
    target = abs(float(distance_m))
    orientation = board.CW if distance_m >= 0 else board.CCW
    speed_pct = float(speed_pct)
    circ = math.pi * float(wheel_diameter_m/1000)  # wheel circumference

    # Safety clamps
    speed_pct = max(0.0, min(100.0, speed_pct))
    min_speed_pct = max(0.0, min(min_speed_pct, speed_pct))

    board.motor_movement([board.M1, board.M2], orientation, speed_pct)

    # 3) Integrate distance until we hit target
    sL = sR = 0.0
    s_last_change_t = time.monotonic()
    t_prev = s_last_change_t

    try:
        while True:
            time.sleep(poll_dt)
            t_now = time.monotonic()
            dt = t_now - t_prev
            t_prev = t_now

            rpmL, rpmR = board.get_encoder_speed([board.M1, board.M2])

            # Convert RPM -> revolutions advanced over dt.
            # Use absolute in case encoder sign differs from motor orientation wiring.
            revL = abs(rpmL) * dt / 60.0
            revR = abs(rpmR) * dt / 60.0

            # Distance per wheel
            sL += revL * circ
            sR += revR * circ

            # Chassis distance = average of wheel distances
            s = 0.5 * (sL + sR)

            # Watchdog: detect if we’ve effectively stopped moving
            if (revL + revR) > 0:
                s_last_change_t = t_now
            elif (t_now - s_last_change_t) > watchdog_s:
                # Not moving for too long -> break to avoid hanging
                break

            remaining = target - s
            if remaining <= 0:
                break

            # 4) Simple taper near the goal to reduce overshoot
            if remaining < slow_down_window_m:
                scaled = speed_pct * (remaining / slow_down_window_m)
                cmd_speed = max(min_speed_pct, scaled)
                board.motor_movement([board.M1, board.M2], orientation, cmd_speed)

    finally:
        board.motor_stop(board.ALL)

    return 0.5 * (sL + sR)


if __name__ == "__main__":
  

    board_detect()    # If you forget address you had set, use this to detected them, must have class instance

    # Set board controler address, use it carefully, reboot module to make it effective
    '''
    board.set_addr(0x10)
    if board.last_operate_status != board.STA_OK:
    print("set board address faild")
    else:
    print("set board address success")
    '''

    while board.begin() != board.STA_OK:    # Board begin and check board status
        print_board_status()
        print("board begin faild")
        time.sleep(2)
    print("board begin success")

    board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
    # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
    board.set_encoder_reduction_ratio(board.ALL, 50)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

    board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

    drive_distance(board, distance_m=0.3, speed_pct=50)


    print("stop all motor")
    board.motor_stop(board.ALL)   # stop all DC motor
    print_board_status()
    time.sleep(4)
