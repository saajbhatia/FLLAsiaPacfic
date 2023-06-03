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
        return correct - (curr - 360)
    elif curr - correct < -180:
        return correct - (curr + 360)
    else:
        return correct - curr

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
        if abs(calDiff(hub.motion_sensor.get_yaw_angle(), deg)) <= 3:
            break
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)

def left_abs_turning(hub, robot, deg, speed):
    distOfWheels = 39.0
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', -100, speed)


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



'''
DO NOT CHANGE
'''

# initAngle = 0
def powerplant_and_tv(hub, robot, flipper, back_flipper, flipperInit):
    #Go to pwr plant
    abs_flip_turn(flipper, 63, 50, flipperInit)
    highspeed_pid(hub, robot, 50, 80, 0)
    abs_turning(hub, robot, -6, 50)
    abs_flip_turn(flipper, 0, 100, flipperInit)

    #Turn right 35, and go diagonal
    abs_turning(hub, robot, 30, 50)
    pid(hub, robot, 3, 50, 30)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 50)

    #Go more B4 pushing down
    pid(hub, robot, 5, 50, 0)
    abs_flip_turn(flipper, 90, 50, flipperInit)

    #Go HOME
    highspeed_pid(hub, robot, 76, -100, 0)

    #Turn for TV
    abs_turning(hub, robot, 90, 50)

    #Tv mission go forward and come back
    pid(hub, robot, 33, 50, 90)
    pid(hub, robot, 40, -100, 90)
    abs_flip_turn(flipper, 0, 100, flipperInit)
    return

def car_windmill(hub, robot, flipper, flipperInit):
    hub.motion_sensor.reset_yaw_angle()

    abs_turning(hub, robot, -46, 30)

    #Dump + Grab
    highspeed_pid(hub, robot, 65, 80, -46)
    abs_flip_turn(flipper, 90, 30, flipperInit)
    highspeed_pid(hub, robot, 18, 80, -46)

    #Do Car
    abs_turning(hub, robot, -21, 30)
    abs_flip_turn(flipper, 0, 30, flipperInit)
    time.sleep(0.1)
    abs_flip_turn(flipper, 90, 30, flipperInit)
    abs_turning(hub, robot, -50, 30)

    #Back from car
    highspeed_pid(hub, robot, 31, -50, -48)
    abs_flip_turn(flipper, 90, 30, flipperInit)

    #Turn for windmill
    fast_turning(hub, robot, 60, 30)
    wait_for_seconds(0.1)
    abs_turning(hub, robot, 45, 30)

    #Do Windmill
    highspeed_pid(hub, robot, 29, 80, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 6.5, -50, 45)
    pid(hub, robot, 6, 30, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 6, -50, 45)
    pid(hub, robot, 8, 30, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 13, -50, 45)

    #Put units on floor
    left_abs_turning(hub, robot, -145, 50)
    abs_turning(hub, robot, -145, 50)
    wait_for_seconds(1)
    flipper.run_for_degrees(-90, 30)

    #Go home
    fast_turning(hub, robot, -215, 45)
    highspeed_pid(hub, robot, 65, 100, -205)

    print("Time for Total Time: " + str(time.time()-currentTime))


def test():
    abs_flip_turn(flipper, 90, 30, flipperInit)

currentTime = time.time()
powerplant_and_tv(hub, robot, flipper, back_flipper, int(flipper.get_position()))
car_windmill(hub, robot, flipper, back_flipper, int(flipper.get_position()))
#test()
print(time.time()-currentTime)
