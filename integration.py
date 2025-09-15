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
from cameraModules.processing_module import process_frame, Obstacle
from cameraModules.camera_module import Camera
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
              max_duty=65, min_duty=35,
              kp=110.0,
              timeout_s=6.0,
              eps_deg=2.0,
              rpm_alpha=0.4,
              do_probe=True,          # <â€” key: normalise encoder delta sign
              debug=False):

    # --- geometry ---
    Rw_mm = WHEEL_DIAMETER_MM / 2.0
    L_mm  = TRACK_WIDTH_MM
    theta_rad = math.radians(angle_deg)
    target_revs_diff = (theta_rad * L_mm) / (2.0 * math.pi * Rw_mm)
    eps_revs_diff    = (math.radians(eps_deg) * L_mm) / (2.0 * math.pi * Rw_mm)

    # Nominal spin directions (keep "one fwd / one back" ALL the time)
    # +angle (left/CCW): RIGHT forward, LEFT backward
    # -angle (right/CW): LEFT forward, RIGHT backward
    left_nominal_sign, right_nominal_sign = ( -1, +1 ) if angle_deg >= 0 else ( +1, -1 )

    # Always start from rest
    board.motor_stop(board.ALL)
    sleep(0.05)

    # ---- PROBE: figure out the sign of (rpmR - rpmL) for THIS pattern ----
    # dir_gain makes "progress" positive when turning the requested way
    dir_gain = 1.0
    if do_probe:
        probe_duty = max(min_duty + 5, 30)
        set_wheel(LEFT_MOTOR_ID,  left_nominal_sign  * probe_duty, LEFT_FORWARD_ORIENT)
        set_wheel(RIGHT_MOTOR_ID, right_nominal_sign * probe_duty, RIGHT_FORWARD_ORIENT)
        sleep(0.20)
        rpm_L0, rpm_R0 = read_rpm()
        board.motor_stop(board.ALL)
        sleep(0.05)
        delta0 = (rpm_R0 - rpm_L0)
        # If delta is <= 0, measured progress has opposite sign; flip the measurement sign
        dir_gain = 1.0 if delta0 > 0 else -1.0
        if debug:
            print(f"[probe] rpm_L={rpm_L0:.1f}, rpm_R={rpm_R0:.1f}, delta={delta0:.1f}, dir_gain={dir_gain}")

    # Optional "kick" to unstick
    kick_duty = max(min_duty, int(0.6 * max_duty))
    set_wheel(LEFT_MOTOR_ID,  left_nominal_sign  * kick_duty, LEFT_FORWARD_ORIENT)
    set_wheel(RIGHT_MOTOR_ID, right_nominal_sign * kick_duty, RIGHT_FORWARD_ORIENT)
    sleep(0.15)

    # --- integrators / filters ---
    revs_diff_norm = 0.0   # normalised "progress" (always increases for requested direction)
    t_prev = monotonic()
    t_start = t_prev
    rpmL_f = None
    rpmR_f = None

    # Control loop
    while True:
        t_now = monotonic()
        dt = t_now - t_prev
        t_prev = t_now
        if dt <= 0:
            dt = 1e-3

        rpm_L, rpm_R = read_rpm()
        if rpmL_f is None:
            rpmL_f, rpmR_f = rpm_L, rpm_R
        else:
            rpmL_f = (1 - rpm_alpha) * rpmL_f + rpm_alpha * rpm_L
            rpmR_f = (1 - rpm_alpha) * rpmR_f + rpm_alpha * rpm_R

        # Normalised progress (flip sign if probe said it's inverted)
        revs_diff_norm += (dir_gain * (rpmR_f - rpmL_f) / 60.0) * dt

        err = target_revs_diff - revs_diff_norm

        # stop conditions
        if abs(err) <= max(eps_revs_diff, 1e-4):
            break
        if (t_now - t_start) > timeout_s:
            if debug:
                print("[turnAngle] timeout")
            break

        # P-control for duty magnitude
        raw = kp * err
        mag = max(min(abs(raw), max_duty), min_duty)

        # Preserve in-place pattern at all times.
        # If we overshoot (err < 0), both commands flip sign together,
        # still keeping one fwd, one back.
        s = 1.0 if raw >= 0 else -1.0
        left_cmd  = left_nominal_sign  * s * mag
        right_cmd = right_nominal_sign * s * mag

        set_wheel(LEFT_MOTOR_ID,  left_cmd,  LEFT_FORWARD_ORIENT)
        set_wheel(RIGHT_MOTOR_ID, right_cmd, RIGHT_FORWARD_ORIENT)

        sleep(0.05)

    # Stop + short brake to kill inertia
    board.motor_stop(board.ALL)
    sleep(0.03)
    # Small opposite pulse (brake)
    brake = max(0, min(min_duty, 30))
    if brake > 0:
        set_wheel(LEFT_MOTOR_ID,  -left_nominal_sign  * brake, LEFT_FORWARD_ORIENT)
        set_wheel(RIGHT_MOTOR_ID, -right_nominal_sign * brake, RIGHT_FORWARD_ORIENT)
        sleep(0.05)
        board.motor_stop(board.ALL)

    # Report achieved angle using the normalised progress
    achieved_theta_rad = (revs_diff_norm * (2.0 * math.pi * Rw_mm)) / L_mm
    return math.degrees(achieved_theta_rad)

def turnAngle_seconds(angle, duty=100):
   # 3 seconds at duty cycle of 100 is approximately 250 degrees
   # therefore 1.083 seconds at duty cycle 100 for 90 degrees
   # 0.012 seconds per degree

    tConst = 0.012

    timeDelta = abs(angle) * tConst

    if angle > 0:
        direction = 'right'
    else:
       direction = 'left'

    if direction == 'right':
        board.motor_movement([board.M1], board.CW, duty)
        board.motor_movement([board.M2], board.CW, duty)
    elif direction == 'left':
       board.motor_movement([board.M1], board.CCW, duty)
       board.motor_movement([board.M2], board.CCW, duty)

    sleep(timeDelta)

    board.motor_stop(board.ALL)



# shelf helpers
def calcShelfMarkersRB(obstacles):
    shelfMarkers = Obstacle.filter_by_station_id(obstacles,1)
    shelfMarkerRB = {0: None, 1: None, 2: None}
    if shelfMarkers:
        for shelf in shelfMarkers:
            if shelf.type_name != "Picking Station":
                shelfMarkerRB[shelf.station_id] = (shelf.distance, shelf.bearing[0])
    return shelfMarkerRB



TARGET_SHELF_INDEX = 2
if TARGET_SHELF_INDEX > -1 and TARGET_SHELF_INDEX < 2:
   TARGET_ROW_INDEX = 0
elif TARGET_SHELF_INDEX > 1 and TARGET_SHELF_INDEX < 4:
   TARGET_ROW_INDEX = 1
elif TARGET_SHELF_INDEX > 3 and TARGET_SHELF_INDEX < 6:
   TARGET_ROW_INDEX = 2

TARGET_BAY = 1
BEARING_LIMIT = 0.1


def calcBayDistance():
   reverseBayID = abs(TARGET_BAY-4)
   distance = (reverseBayID-1)*26 + 13
   return distance/100


   


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

        robotMode = RobotMode.SEARCH_MARKER



        while True:
            frame = camera.capture_frame()
            obstacles, trajectory = process_frame(frame)

            shelfMarkersRB = calcShelfMarkersRB(obstacles)


            if robotMode == RobotMode.DEBUG_TEST:
                # testAngle = turnAngle(90, max_duty=80, min_duty=40, kp=120, timeout_s=6)
                # print("Requested 90Â°, achieved ~{:.1f}Â°".format(testAngle))
                # right is positive, left is negative
                turnAngle_seconds(-90)
                robotMode = RobotMode.DEBUG_STOP

            if robotMode == RobotMode.DEBUG_STOP:
                board.motor_stop(board.ALL)
            
            if robotMode == RobotMode.SEARCH_MARKER:
               if shelfMarkersRB[TARGET_ROW_INDEX]:
                  rowmB = shelfMarkersRB[TARGET_ROW_INDEX][1]
                  rowmD = shelfMarkersRB[TARGET_ROW_INDEX][0]
                  if rowmB > 0:
                     direction = 'right'
                  else:
                     direction = 'left'
                  if rowmB > abs(BEARING_LIMIT):
                    if direction == 'right':
                       board.motor_movement([board.M1], board.CW, 50)
                       board.motor_movement([board.M2], board.CW, 50)
                    elif direction == 'left':
                       board.motor_movement([board.M1], board.CCW, 50)
                       board.motor_movement([board.M2], board.CCW, 50)
                  else:
                     board.motor_stop(board.ALL)
                     robotMode = RobotMode.DRIVE_TO_BAY
               else:
                  board.motor_movement([board.M1], board.CW, 50)
                  board.motor_movement([board.M2], board.CW, 50)

            if robotMode == RobotMode.DRIVE_TO_BAY:
               if shelfMarkersRB[TARGET_ROW_INDEX]:
                  rowmB = shelfMarkersRB[TARGET_ROW_INDEX][1]
                  rowmD = shelfMarkersRB[TARGET_ROW_INDEX][0]
                  if rowmD > calcBayDistance():
                     board.motor_movement([board.M1], board.CCW, 50)
                     board.motor_movement([board.M2], board.CW, 50)
                  else:
                     board.motor_stop(board.ALL)
                     robotMode = RobotMode.DEBUG_STOP
                  

    except KeyboardInterrupt:
        print("\nShutting down...")
        board.motor_stop(board.ALL)
    except Exception as e:
       board.motor_stop(board.ALL)
       print(f"\nAn error occurred: {e}")
       traceback.print_exc()
    finally:
        camera.close()
        cv2.destroyAllWindows()



