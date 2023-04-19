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
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
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
    flipper.run_for_degrees(90, 90)
    pid(hub, robot, 51, 30, 0)
    #flipping platforn run
    for i in range(3):
        flipper.run_for_degrees(-30, 80)
        flipper.run_for_degrees(25, 80)
    back_flipper.run_for_degrees(-70, 45)
    abs_turning(hub, robot, 40, 15)
    flipper.run_for_degrees(-95, 90)
    pid(hub, robot, 36, 30, 40)
    abs_turning(hub, robot, 90, 15)
    #back flip goes down to collect cubes
    back_flipper.run_for_degrees(-100, 50)
    pid(hub, robot, 30, 30, 90)
    back_flipper.run_for_degrees(60, 50)
    abs_turning(hub, robot, 187, 15)
    back_flipper.run_for_degrees(-10, 50)
    pid(hub, robot, 21, -30, -173)
    pid(hub, robot, 21, 30, -173)
    flipper.run_for_degrees(90, 90)

plat()
print(time.time()-startTime)
