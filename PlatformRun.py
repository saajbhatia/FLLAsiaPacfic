'''
DO NOT CHANGE
'''
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import math
import time

def highspeed_pid(hub, robot, cm, speed, start_angle):
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
            robot.start(steer, 30)
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
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)

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

startTime = time.time()

def plat():
    #Go to platform
    flipper.run_to_degrees_counted(270-flipper.get_position(), 50)
    back_flipper.set_stop_action('coast')
    abs_backflip_turn(back_flipper, 52, 30, back_flipperInit)
    highspeed_pid(hub, robot, 50.5, 70, 0)
    abs_backflip_turn(back_flipper, 48, 30, back_flipperInit)
    pid(hub, robot, 0.75, 30, 0)
    abs_turning(hub, robot, -6.25, 30)

    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)

    #Flip/do platform
    for i in range(3):
        abs_flip_turn(flipper, 50, 90, flipperInit)
        abs_flip_turn(flipper, 80, 90, flipperInit)

    # #Flip/do water dam mission
    # for i in range(2):
    #    abs_backflip_turn(back_flipper, 104, 70, back_flipperInit)
    #    abs_backflip_turn(back_flipper, 0, 70, back_flipperInit)

    #Go to solar farm energy units
    abs_turning(hub, robot, 39, 50)

    abs_flip_turn(flipper, 0, 50, flipperInit)
    highspeed_pid(hub, robot, 31, 80, 39)
    return
    abs_turning(hub, robot, 90, 50)
    #pid(hub, robot, 0.5, 25, 90)
    #Put back flip down to collect cylinders
    abs_backflip_turn(back_flipper, -115, 30, back_flipperInit)

    highspeed_pid(hub, robot, 32, 80, 90)
    #abs_turning(hub, robot, 110, 40)

    #Code for HIGH FIVE and NRG collection and HYDRO DAM collection
    #abs_turning(hub,robot,110,55)
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)
    pid(hub, robot, 4, -40, 90)
    abs_turning(hub, robot, 180, 40)
    pid(hub, robot, 2, 30, 180)
    back_flipper.set_stop_action('hold')
    abs_backflip_turn(back_flipper, -65, 30, back_flipperInit)

    #Do high five and collect units
    highspeed_pid(hub, robot, 24, -30, 180)
    back_flipper.set_stop_action('coast')

    print(hub.motion_sensor.get_yaw_angle())


    #Go forward
    highspeed_pid(hub, robot, 20, 30, 180)
    abs_turning(hub, robot, 195, 40)

    #Catch nrg
    #waitUntilTap(hub)
    flipper.run_for_degrees(70, 60)
    abs_turning(hub, robot, 165, 40)
    highspeed_pid(hub, robot, 51, 80, 165)

    #Turn 90 degrees clockwise
    abs_turning(hub, robot, 260, 40)
    #back_flipper.run_for_degrees(120, 50)
    abs_backflip_turn(back_flipper, -10, 30, back_flipperInit)
    pid(hub, robot, 85, 90, 260)
    #back_flipper.run_for_degrees(-10, 50)
    #highspeed_pid(hub, robot, 40, 90, 255)
    
    abs_flip_turn(flipper, 0, 55, flipperInit)

    return

    '''
    WORKS UNTIL HERE
    '''

    #Turn 17 deg
    abs_turning(hub, robot, 197, 40)

    pid(hub, robot, 5, 30, 197)
    pid(hub, robot, 4, -30, 197)

    #Catch nrg
    flipper.run_for_degrees(70, 60)


    abs_turning(hub, robot, 170, 40)
    highspeed_pid(hub, robot, 53, 70, 170)

    #Turn 90 degrees clockwise
    abs_turning(hub, robot, 255, 40)
    back_flipper.run_for_degrees(105, 50)
    highspeed_pid(hub, robot, 32, 70, 255)
    back_flipper.run_for_degrees(-60, 50)
    highspeed_pid(hub, robot, 40, 90, 255)

def test():
    highspeed_pid(hub, robot, 12.5, -50, 0)

def dino_run():
    highspeed_pid(hub, robot, 129, 70, 0)
    abs_turning(hub, robot, 40, 50)
    for i in range (2):
        flipper.run_for_degrees(60, 20)
        flipper.run_for_degrees(-45, 20)
    abs_turning(hub, robot, 33, 50)
    pid(hub, robot, 2, 30, 33)
    flipper.run_for_degrees(60, 60)
    abs_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 50, 70, 0)
    return
    fast_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 45, 80, 0)
    wait_for_seconds(1.5)
    highspeed_pid(hub, robot, 19, -80, 0)
    fast_turning(hub, robot, 90, 50)
    highspeed_pid(hub, robot, 25, -80, 90)
    flipper.run_for_degrees(-80, 90)
    hub.motion_sensor.reset_yaw_angle()
    plat()

def dino_only_collect_water_run():
    highspeed_pid(hub, robot, 135.5, 70, 0)
    abs_turning(hub, robot, 47, 50)
    flipper.run_for_degrees(85, 60)
    abs_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 40, 70, 0)

def plat_variation():
    flipper.run_for_degrees(-30, 30)
    abs_turning(hub, robot, -38, 40)
    pid(hub, robot, 40, 40, -38)
    flipper.run_for_degrees(-50, 30)
    pid(hub, robot, 13, -40, -38)
    abs_turning(hub, robot, -90, 40)
    flipper.run_for_degrees(80, 30)
    pid(hub, robot, 26, 40, -90)

def dump():
    hub.motion_sensor.reset_yaw_angle()
    abs_turning(hub, robot, 3, 30)
    highspeed_pid(hub, robot, 73, 80, 3)
    abs_turning(hub, robot, 0, 30)
    flipper.run_for_rotations(0.1, 30)
    flipper.run_for_rotations(0.1, -50)
    pid(hub, robot, 2, -40, 0)
    abs_flip_turn(flipper, 70, 50, flipperInit)
    #abs_turning(hub, robot, 10, 30)
    pid(hub, robot, 71, -90, 10)
    #abs_turning(hub, robot, 0, 30)
    abs_flip_turn(flipper, 0, 50, flipperInit)

'''
def reservoir():
    highspeed_pid(hub, robot, 15, 70, 0)
    abs_turning(hub, robot, 45, 30)
    highspeed_pid(hub, robot, 75, 70, 45)

    abs_turning(hub, robot, 90, 30)
    highspeed_pid(hub, robot, 20, 70, 90)
    abs_turning(hub, robot, 140, 30)

    pid(hub, robot, 25, 40, 140)
    #used to be 16
    pid(hub, robot, 13.5, -30, 140)

    print(hub.motion_sensor.get_yaw_angle())

    diff = 140 - hub.motion_sensor.get_yaw_angle()
    hub.motion_sensor.reset_yaw_angle()
    abs_turning(hub, robot, 93 + diff, 30)

    print(hub.motion_sensor.get_yaw_angle())

    pid(hub, robot, 9.5, 40, 93 + diff)
    #abs_turning(hub, robot, 93 + diff, 50)

    print(hub.motion_sensor.get_yaw_angle())

    flipper.run_for_degrees(60, 10)

    pid(hub, robot, 5, -40, 93 + diff)
    flipper.run_for_degrees(60, -10)
    pid(hub, robot, 10, -40, 93 + diff)

    return

'''

#dino_run()
plat()
print("Time: " + str(time.time()-startTime))
waitUntilTap(hub)
dump()
waitUntilTap(hub)
reservoir()

#highfive_collectnrg()


print("Time: " + str(time.time()-startTime))
