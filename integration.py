from __future__ import print_function
import sys
import os
sys.path.append("../")

from time import sleep, monotonic
import types
import traceback
from enum import IntEnum
import math

from DFRobot_RaspberryPi_DC_Motor import THIS_BOARD_TYPE, DFRobot_DC_Motor_IIC as Board

'''
subsystem integration modules
'''
from openCV.processing_module import process_frame, Obstacle
from openCV.camera_module import Camera
import cv2

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


class RobotMode(IntEnum):
   SEARCH_MARKER = 1
   DRIVE_TO_BAY = 2
   ARRIVE_BAY = 3
   DROP_ITEM = 4
   DEBUG_STOP = 5
   DEBUG_TEST = 6

# physical measurements in mm
WHEEL_DIAMETER_MM = 30
TRACK_WIDTH_MM = 170

LEFT_MOTOR_ID =  board.M1
RIGHT_MOTOR_ID = board.M2
LEFT_FORWARD_ORIENT = board.CW
RIGHT_FORWARD_ORIENT = board.CCW

def opposite(orient):
    return board.CCW if orient == board.CW else board.CW

def set_wheel(motor_id, duty_signed, forward_orient):
    duty = abs(int(duty_signed))
    if duty == 0:
        board.motor_stop([motor_id])
        return
    orient = forward_orient if duty_signed >= 0 else opposite(forward_orient)
    board.motor_movement([motor_id], orient, duty)


def read_rpm(left_id=LEFT_MOTOR_ID, right_id=RIGHT_MOTOR_ID):
    rpms = board.get_encoder_speed([left_id, right_id])
    return float(rpms[0]), float(rpms[1])


def calibrate_forward_orientations(test_duty=40, sample_s=0.25):
    global LEFT_FORWARD_ORIENT, RIGHT_FORWARD_ORIENT
    # Left
    board.motor_movement([LEFT_MOTOR_ID], board.CW, test_duty)
    board.motor_stop([RIGHT_MOTOR_ID])
    sleep(sample_s)
    l_rpm, _ = read_rpm()
    LEFT_FORWARD_ORIENT = board.CW if l_rpm > 0 else board.CCW
    board.motor_stop([LEFT_MOTOR_ID])

    # Right
    board.motor_movement([RIGHT_MOTOR_ID], board.CW, test_duty)
    board.motor_stop([LEFT_MOTOR_ID])
    sleep(sample_s)
    _, r_rpm = read_rpm()
    RIGHT_FORWARD_ORIENT = board.CW if r_rpm > 0 else board.CCW
    board.motor_stop([RIGHT_MOTOR_ID])

    # small pause
    sleep(0.2)

def turnAngle(angle_deg,
              max_duty=55, min_duty=25,
              kp=120.0,                 # proportional gain -> duty ≈ kp * remaining_revs_diff
              timeout_s=6.0,
              eps_deg=1.0):
    circumference_mm = math.pi * WHEEL_DIAMETER_MM
    L_mm = TRACK_WIDTH_MM

    theta_rad = math.radians(angle_deg)
    target_revs_diff = (theta_rad * L_mm) / (2.0 * math.pi * (WHEEL_DIAMETER_MM / 2.0)) 
    eps_revs_diff = (math.radians(eps_deg) * L_mm) / (2.0 * math.pi * (WHEEL_DIAMETER_MM / 2.0))

    revs_diff = 0.0
    t_prev = monotonic()
    t_start = t_prev

    while True:
        t_now = monotonic()
        dt = t_now - t_prev
        t_prev = t_now

        rpm_L, rpm_R = read_rpm()
        # (rev_R - rev_L) increment
        revs_diff += ((rpm_R - rpm_L) / 60.0) * dt

        # Remaining error
        err = target_revs_diff - revs_diff

        # Done?
        if abs(err) <= max(eps_revs_diff, 1e-4):
            break

        if (t_now - t_start) > timeout_s:
            break

        # Proportional duty command (symmetric spin)
        raw = kp * err
        duty = max(min(abs(raw), max_duty), min_duty)
        sign = 1.0 if raw >= 0 else -1.0

        # To increase (rev_R - rev_L), spin RIGHT forward and LEFT backward (and vice versa to decrease)
        right_cmd =  sign * duty
        left_cmd  = -sign * duty

        set_wheel(LEFT_MOTOR_ID,  left_cmd,  LEFT_FORWARD_ORIENT)
        set_wheel(RIGHT_MOTOR_ID, right_cmd, RIGHT_FORWARD_ORIENT)

        sleep(0.05)

    # Hard stop
    board.motor_stop(board.ALL)
    # Optional short brake dwell
    sleep(0.05)
    # Return the actual achieved angle estimate (useful for debugging)
    achieved_theta_rad = (revs_diff * (2.0 * math.pi * (WHEEL_DIAMETER_MM / 2.0))) / L_mm
    return math.degrees(achieved_theta_rad)


   


if __name__ == "__main__":
    try:
        board_detect()


        while board.begin() != board.STA_OK:    # Board begin and check board status
            print_board_status()
            print("board begin faild")
            sleep(2)
        print("board begin success")

        board.set_encoder_enable(board.ALL)                 # Set selected DC motor encoder enable
        # board.set_encoder_disable(board.ALL)              # Set selected DC motor encoder disable
        board.set_encoder_reduction_ratio(board.ALL, 43)    # Set selected DC motor encoder reduction ratio, test motor reduction ratio is 43.8

        board.set_moter_pwm_frequency(1000)   # Set DC motor pwm frequency to 1000HZ

        # initialise camera processing
        camera = Camera()

        # Target frequency in Hz
        target_freq = 20.0
        period = 1.0 / target_freq

        #robotMode = RobotMode.SEARCH_MARKER

        robotMode = RobotMode.DEBUG_TEST



        while True:
            frame = camera.capture_frame()
            obstacles, trajectory = process_frame(frame)

            if robotMode == RobotMode.DEBUG_TEST:
                testAngle = turnAngle(90, maxDuty=80, minDuty=40, kp=120, timeout_s=6)
                print("Requested 90°, achieved ~{:.1f}°".format(testAngle))
                robotMode = RobotMode.DEBUG_STOP

            if robotMode == RobotMode.DEBUG_STOP:
                board.motor_movement([board.M1], board.CW, 0)  
                board.motor_movement([board.M2], board.CCW, 0)
    except KeyboardInterrupt:
        print("\nShutting down...")
        board.motor_movement([board.M1], board.CW, 0)  
        board.motor_movement([board.M2], board.CCW, 0)
    except Exception as e:
       board.motor_movement([board.M1], board.CW, 0)  
       board.motor_movement([board.M2], board.CCW, 0)
       print(f"\nAn error occurred: {e}")
       traceback.print_exc()
    finally:
        camera.close()
        cv2.destroyAllWindows()


