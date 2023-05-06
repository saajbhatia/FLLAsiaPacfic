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
    while abs(motor.get_degrees_counted())<=degrees_needed*0.2:
        GSPK = 1.7
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*GSPK
        if speed < 0:
            steer *= -1
        if speed < 0:
            robot.start(int(steer), -30)
        else:
            robot.start(int(steer), 30)
        #print(steer)
        robot.start(int(steer),speed)
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

def fast_turning(hub, robot, deg, speed):
    distOfWheels = 38.0
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(17.5, 'cm')

    flipper = Motor('D')
    #flipper.set_stop_action('hold')

    back_flipper = Motor('A')
    back_flipper.set_stop_action('hold')
    return hub, robot, flipper, back_flipper

def waitUntilTap(hub):
    while True:
        if hub.motion_sensor.wait_for_new_gesture() == 'tapped':
            break

hub, robot, flipper, back_flipper = __init__()

'''
DO NOT CHANGE
'''

startTime = time.time()
def plat():
    back_flipper.run_for_degrees(74, 30)
    flipper.run_for_degrees(80, 30)
    highspeed_pid(hub, robot, 51.5, 80, 0)
    abs_turning(hub, robot, 0, 20)
    #flipping platform run
    for i in range(3):
        flipper.run_for_rotations(-0.1, 30)
        flipper.run_for_rotations(0.1, 30)
    back_flipper.run_for_degrees(-70, 30)
    
    #Everything after this isn't tested
    abs_turning(hub, robot, 38, 50)
    flipper.run_for_degrees(-140, 30)
    pid(hub, robot, 36, 50, 38)
    abs_turning(hub, robot, 90, 50)
    return
    #back flip goes down to collect cylinders
    back_flipper.run_for_degrees(-100, 50)
    pid(hub, robot, 30, 50, 90)
    back_flipper.run_for_degrees(60, 50)
    abs_turning(hub, robot, 180, 50)
    back_flipper.run_for_degrees(-10, 50)
    pid(hub, robot, 21, -50, 180)
    pid(hub, robot, 21, 50, 180)
    abs_turning(hub, robot, 195, 50)
    flipper.run_for_degrees(130, 90)
    abs_turning(hub, robot, 150, 50)
def test():
    back_flipper.run_for_degrees(160, 40)

def dino_run():
    highspeed_pid(hub, robot, 129, 80, 0)
    fast_turning(hub, robot, 40, 50)
    flipper.run_for_degrees(80, 90)
    fast_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 45, 80, 0)
    wait_for_seconds(1.5)
    highspeed_pid(hub, robot, 19, -80, 0)
    fast_turning(hub, robot, 90, 50)
    highspeed_pid(hub, robot, 25, -80, 90)
    flipper.run_for_degrees(-80, 90)
    hub.motion_sensor.reset_yaw_angle()
    plat()



#dino_run()
plat()
print(time.time()-startTime)
