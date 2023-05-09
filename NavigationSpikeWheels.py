'''
DO NOT CHANGE
'''
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import math
import time

def pid(hub, robot, cm, speed, start_angle):
    print('*******************')
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/17.8
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 2
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*GSPK
        if speed < 0:
            steer *= -1
        #print(steer)
        robot.start(int(steer),speed)
        #wait_for_seconds(0.1)
    robot.stop()

def highspeed_pid(hub, robot, cm, speed, start_angle):
    print('*******************')
    motor = Motor('F')
    motor1 = Motor('B')
    motor.set_degrees_counted(0)
    motor1.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/17.8

    while abs(((abs(motor.get_degrees_counted())+abs(motor1.get_degrees_counted())))/2)<=degrees_needed*0.15:
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*2
        if speed < 0:
            steer *= -1
        if speed < 0:
            robot.start(int(steer), -30)
        else:
            robot.start(int(steer), 30)
        robot.start(int(steer),speed)
    while abs(((abs(motor.get_degrees_counted())+abs(motor1.get_degrees_counted())))/2)<=degrees_needed*0.70:
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*6
        if speed < 0:
            steer *= -1
        robot.start(int(steer),speed)
    while abs(((abs(motor.get_degrees_counted())+abs(motor1.get_degrees_counted())))/2)<=degrees_needed:
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*2
        if speed < 0:
            steer *= -1
        if speed < 0:
            robot.start(int(steer), -30)
        else:
            robot.start(int(steer), 30)
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
    distOfWheels = 38.0
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)

    print('Total time is', time.time()-startTime)

def fast_turning(hub, robot, deg, speed):
    distOfWheels = 38.0
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(17.5, 'cm')

    flipper = Motor('D')
    flipper.set_stop_action('hold')

    back_flipper = Motor('A')
    return hub, robot, flipper, back_flipper

def waitUntilTap(hub):
    while True:
        if hub.motion_sensor.wait_for_new_gesture() == 'tapped':
            break

hub, robot, flipper, back_flipper = __init__()
highspeed_pid(hub, robot, 64, 70, 0)
