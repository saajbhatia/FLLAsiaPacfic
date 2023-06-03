'''
DO NOT CHANGE
'''
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import math
import time

def highspeed_pid(hub, robot, cm, speed, start_angle):
    print('*******************')
    motor = Motor('F')
    motor.set_degrees_counted(0)
    degrees_needed = abs(cm) * 360/17.8
    prevError = 0
    while abs(motor.get_degrees_counted())<=degrees_needed*0.10:
        error = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)
        #print(error, error*2+prevError*1, prevError)
        wait_for_seconds(0.1)
        steer = error*2+prevError*1
        if speed < 0:
            steer *= -1
        if speed > 0:
            robot.start(steer,30)
        else:
            robot.start(steer, -30)
        prevError = error
    while abs(motor.get_degrees_counted())<=degrees_needed*0.8:
        error = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)
        #print(error, error*2+prevError*1, prevError)
        steer = error*2+prevError*1
        if speed < 0:
            steer *= -1
        robot.start(steer,speed)
        prevError = error
    while abs(motor.get_degrees_counted())<=degrees_needed:
        error = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)
        #print(error, error*2+prevError*1, prevError)
        wait_for_seconds(0.1)
        steer = error*2+prevError*1
        if speed < 0:
            steer *= -1
        if speed > 0:
            robot.start(steer,30)
        else:
            robot.start(steer, -30)
        prevError = error
    robot.stop()

def pid(hub, robot, cm, speed, start_angle):
    print('*******************')
    motor = Motor('F')
    motor.set_degrees_counted(0)
    degrees_needed = abs(cm) * 360/17.8
    prevError = 0
    while abs(motor.get_degrees_counted())<=degrees_needed:
        error = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)
        #print(error, error*2+prevError*1, prevError)
        steer = error*2+prevError*1
        if speed < 0:
            steer *= -1
        robot.start(steer,speed)
        prevError = error

    robot.stop()

def calDiff(curr, correct):
    if curr - correct > 180:
        return correct - (curr - 360) + initAngle
    elif curr - correct < -180:
        return correct - (curr + 360) + initAngle
    else:
        return correct - curr + initAngle

def calDiffFlip(num):
    while True:
        if num < 0:
            num += 360
        elif num > 359:
            num -= 360
        else:
            return num

def abs_turning(hub, robot, deg, speed):
    startTime = time.time()
    distOfWheels = 39.0
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)

    print('Total time is', time.time()-startTime)

def fast_turning(hub, robot, deg, speed):
    distOfWheels = 39.0
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(17.5, 'cm')

    flipper = Motor('D')
    flipper.set_stop_action('brake')

    back_flipper = Motor('A')
    back_flipper.set_stop_action('coast')
    return hub, robot, flipper, back_flipper

def waitUntilTap(hub):
    hub.right_button.wait_until_pressed()

hub, robot, flipper, back_flipper = __init__()

def abs_flip_turn(flipper, correct, speed, flipperInit):
    flipper.run_to_position(calDiffFlip(flipperInit+correct), 'shortest path', speed)

def abs_backflip_turn(back_flipper, correct, speed, back_flipperInit):
    back_flipper.run_to_position(calDiffFlip(back_flipperInit+correct), 'shortest path', speed)

flipperInit = int(flipper.get_position())
back_flipperInit = int(back_flipper.get_position())
