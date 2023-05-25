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
    initAngle = 90
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

'''
DO NOT CHANGE
'''

initAngle = 0
def mission(hub, robot, flipper, back_flipper, flipperInit):

    #Tv mission go forward and come back
    pid(hub, robot, 32, 30, 0)
    highspeed_pid(hub, robot, 44.5, -80, 0)
    abs_turning(hub, robot, -90, 30)

    abs_flip_turn(flipper, -20, 30, flipperInit)


    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    highspeed_pid(hub, robot, 62, 80, -90)
    abs_flip_turn(flipper, -90, 50, flipperInit)

    #Turn right 35
    abs_turning(hub, robot, -45, 30)
    pid(hub, robot, 12.5, 50, -45)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, -90, 30)
    flipper.run_for_degrees(90, 60)
    flipper.run_for_degrees(-5, -30)

    #Go HOME
    highspeed_pid(hub, robot, 80, -80, -90)
    car_windmill(hub, robot, flipper, back_flipper, flipperInit)

def car_windmill(hub, robot, flipper, back_flipper, flipperInit):
    
    waitUntilTap(hub)
    flipper.run_for_degrees(-85, 30)

    abs_turning(hub, robot, -48, 30)

    #Dump + Grab
    highspeed_pid(hub, robot, 65, 80, -48)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    highspeed_pid(hub, robot, 18, 80, -48)

    #Car
    abs_turning(hub, robot, -17, 30)
    abs_flip_turn(flipper, -90, 30, flipperInit)


    #Car mission
    abs_flip_turn(flipper, 0, 30, flipperInit)
    abs_turning(hub, robot, -48, 30)

    #waitUntilTap(hub)
    #back_flipper.run_for_degrees(90, 30)

    #Back from car
    back_flipper.run_for_degrees(110, 30)
    highspeed_pid(hub, robot, 33, -80, -48)

    abs_turning(hub, robot, 20, 10)
    #Turn flipper up b4 doing windmill
    back_flipper.run_for_degrees(110, -30)
    abs_turning(hub, robot, 47, 10)


    #Windmill
    #flipper.run_for_degrees(140, 30)

    highspeed_pid(hub, robot, 18, 80, 47)
    abs_turning(hub, robot, 45, 30)
    for i in range(3):
        pid(hub, robot, 10.5, 30, 45)
        wait_for_seconds(0.5)
        pid(hub, robot, 9.75, -50, 45)

    #waitUntilTap(hub)

    pid(hub, robot, 5, -30, 47)

    #Put units on floor
    fast_turning(hub, robot, -90, 30)
    fast_turning(hub, robot, -135, 45)
    wait_for_seconds(1)
    flipper.run_for_degrees(-90, 30)

    #Go home
    back_flipper.run_for_degrees(110, -30)

    #OG angle -205
    fast_turning(hub, robot, -215, 45)
    back_flipper.run_for_degrees(-90, 30)
    highspeed_pid(hub, robot, 65, 80, -205)


def test():
    flipper.run_for_degrees(-90, 30)

currentTime = time.time()
car_windmill(hub, robot, flipper, back_flipper, flipperInit)
#mission(hub, robot, flipper, back_flipper, flipperInit)
print(time.time()-currentTime)
