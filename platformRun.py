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

'''
DO NOT CHANGE
'''

startTime = time.time()
def plat():
    back_flipper.run_for_degrees(35, 60)
    flipper.run_for_degrees(160, 90)
    pid(hub, robot, 51, 50, 0)
    #flipping platforn run
    for i in range(3):
        flipper.run_for_rotations(-0.1, 80)
        flipper.run_for_rotations(0.1, 80)
    back_flipper.run_for_degrees(-70, 45)
    abs_turning(hub, robot, 38, 50)
    flipper.run_for_degrees(-120, 90)
    pid(hub, robot, 36, 50, 38)
    abs_turning(hub, robot, 90, 50)
    #back flip goes down to collect cylinders
    back_flipper.run_for_degrees(-100, 50)
    pid(hub, robot, 30, 50, 90)
    back_flipper.run_for_degrees(60, 50)
    abs_turning(hub, robot, 180, 50)
    back_flipper.run_for_degrees(-10, 50)
    pid(hub, robot, 21, -50, 180)
    pid(hub, robot, 21, 50, 180)
    flipper.run_for_degrees(110, 90)
    abs_turning(hub, robot, 150, 50)

def test():
    abs_turning(hub, robot, 180, 50)
    print('Done')
    pid(hub, robot, 100, 30, 180)
plat()
print(time.time()-startTime)
