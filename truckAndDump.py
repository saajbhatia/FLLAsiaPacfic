'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import math
import time

def pid(hub, robot, cm, speed, start_angle):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/17.8
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*GSPK
        robot.start(int(steer),speed)
        #wait_for_seconds(0.1)
    robot.stop()

def calDiff(curr, correct):
    if curr - correct > 180:
        return correct - (curr - 360)
    elif curr - correct < -180:
        return correct - (curr + 360)
    else:
        return correct - curr

def abs_turning(hub, robot, deg, speed):
    startTime = time.time()
    distOfWheels = 38.25
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
    print('Total time is', time.time()-startTime)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(17.5, 'cm')

    flipper = Motor('D')
    flipper.set_stop_action('hold')

    back_flipper = Motor('A')
    return hub, robot, flipper, back_flipper

hub, robot, flipper, back_flipper = __init__()
def waitUntilTap(hub):
    while True:
        if hub.motion_sensor.wait_for_new_gesture() == 'freefall':
            print('freefall')
            break 

'''
DO NOT CHANGE
'''

def truck():
    pid(hub, robot, 25.5, 30, 0)
    pid(hub, robot, 40, -30, 0)

    abs_turning(hub, robot, 45, 50)
    flipper.run_for_rotations(-0.25)

def dump():
    waitUntilTap(hub)
    pid(hub, robot, 38, 50, 45)
    abs_turning(hub, robot, 0, 10)
    pid(hub, robot, 48, 50, 0)
    flipper.run_for_rotations(0.12, 50)
    pid(hub, robot, 5, -50, 0)
    flipper.run_for_rotations(0.1)
    pid(hub, robot, 300, -50, 0)
    abs_turning(hub, robot, -90, 20)
    pid(hub, robot, 20, -50, -90)

print('************')

currentTime = time.time()

truck()
dump()

print(time.time()-currentTime)
