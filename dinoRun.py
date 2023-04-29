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
        print(steer)
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
        if hub.motion_sensor.wait_for_new_gesture() == 'tapped':
            break

'''
DO NOT CHANGE
'''

def mission():

    flipper.run_for_degrees(18,-70)

    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    pid(hub, robot, 52.5, 50, 0)
    flipper.run_for_degrees(80,-90)

    #Turn right 35
    abs_turning(hub, robot, 35, 50)
    pid(hub, robot, 8, 50, 34)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 50)
    flipper.run_for_degrees(90, 90)
    flipper.run_for_degrees(-5, -75)

    #Go HOME
    pid(hub, robot, 71, -60, 0)
    abs_turning(hub, robot, 90, 0)

    #go to tv run
    pid(hub, robot, 30, 40, 90)
    pid(hub, robot, 26, -40, 90)

    #Turn to do factory
    abs_turning(hub, robot, 45, 60)

    #Setup
    flipper.run_for_degrees(-140, 30)

    #Dump + Grab
    pid(hub, robot, 75, 70, 45)
    flipper.run_for_degrees(140, 30)
    pid(hub, robot, 14, 70, 45)

    #Car
    abs_turning(hub, robot, 75, 45)
    flipper.run_for_degrees(-140, 30)
    abs_turning(hub, robot, 45, 45)
    pid(hub, robot, 28, -45, 45)
    abs_turning(hub, robot, 137, 45)

    #Windmill
    flipper.run_for_degrees(140, 30)
    pid(hub, robot, 15, 70, 137)
    abs_turning(hub, robot, 137, 45)
    for i in range(3):
        pid(hub, robot, 11.5, 30, 137)
        pid(hub, robot, 10.5, -30, 137)
        wait_for_seconds(0.5)

    #Put units on floor
    abs_turning(hub, robot, -44, 45)
    flipper.run_for_degrees(-140, 30)

    #Go home
    abs_turning(hub, robot, -105, 45)
    pid(hub, robot, 55, 70, -115)

    #Align for dino
    abs_turning(hub, robot, 0, 45)

def test():
    pid(hub, robot, 200, -70, 0)

currentTime = time.time()
mission()
print(time.time()-currentTime)
