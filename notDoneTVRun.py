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
        GSPK = 1.7
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
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/17.8
    while abs(motor.get_degrees_counted())<=degrees_needed*0.8:
        GSPK = 1.7
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*GSPK
        if speed < 0:
            steer *= -1
        #print(steer)
        robot.start(int(steer),speed)
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*GSPK
        if speed < 0:
            steer *= -1
        print(steer)
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

'''
DO NOT CHANGE
'''

def mission():

    flipper.run_for_degrees(18,-50)

    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    highspeed_pid(hub, robot, 53, 80, 0)
    flipper.run_for_degrees(80,-90)

    #Turn right 35
    abs_turning(hub, robot, 30, 50)
    pid(hub, robot, 10, 30, 35)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 50)
    flipper.run_for_degrees(90, 50)
    flipper.run_for_degrees(-5, -50)

    #Go HOME
    highspeed_pid(hub, robot, 71, -80, 0)
    abs_turning(hub, robot, 90, 30)

    #Tv mission go forward and come back
    pid(hub, robot, 29, 50, 90)
    pid(hub, robot, 25, -50, 90)
    flipper.run_for_degrees(-140, 50)
    waitUntilTap(hub)

    #Turn to do factory
    pid(hub, robot, 3, 30, 90)
    abs_turning(hub, robot, 45, 50)

    #Dump + Grab
    highspeed_pid(hub, robot, 75, 80, 45)
    flipper.run_for_degrees(140, 50)
    pid(hub, robot, 8, 80, 45)
    time.sleep(0.1)

    #Car
    abs_turning(hub, robot, 64, 30)
    flipper.run_for_degrees(-140, 50)
    abs_turning(hub, robot, 45, 50)
    pid(hub, robot, 30, -50, 45)
    if flipper.get_position() > 10:
        flipper.run_for_degrees(140, 50)
    abs_turning(hub, robot, 145, 50)
    abs_turning(hub, robot, 137, 50)


    #Windmill
    #flipper.run_for_degrees(140, 30)
    pid(hub, robot, 15, 70, 137)
    abs_turning(hub, robot, 137, 45)
    for i in range(3):
        pid(hub, robot, 10.5, 30, 137)
        pid(hub, robot, 10.5, -30, 137)
        wait_for_seconds(0.5)

    #Put units on floor
    abs_turning(hub, robot, 0, 50)
    abs_turning(hub, robot, -45, 45)
    flipper.run_for_degrees(-140, 30)

    #Go home
    abs_turning(hub, robot, -105, 45)
    highspeed_pid(hub, robot, 55, 80, -115)

    #Align for dino
    abs_turning(hub, robot, 0, 45)

def test():
    pid(hub, robot, 200, -70, 0)

currentTime = time.time()
mission()
print(time.time()-currentTime)
