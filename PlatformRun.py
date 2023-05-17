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
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*4
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
        steer = calDiff(hub.motion_sensor.get_yaw_angle(), start_angle)*4
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
    while True:
        if hub.motion_sensor.wait_for_new_gesture() == 'tapped':
            break

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
    abs_flip_turn(flipper, 80, 50, flipperInit)
    pid(hub, robot, 52.5, 50, 0)
    abs_turning(hub, robot, -4, 30)

    #Flip/do platform
    for i in range(4):
        abs_flip_turn(flipper, 50, 50, flipperInit)
        abs_flip_turn(flipper, 80, 50, flipperInit)

    # #Flip/do water dam mission
    # for i in range(2):
    #    abs_backflip_turn(back_flipper, 104, 70, back_flipperInit)
    #    abs_backflip_turn(back_flipper, 0, 70, back_flipperInit)

    #Go to solar farm energy units
    abs_turning(hub, robot, 39, 50)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    pid(hub, robot, 36, 30, 39)
    abs_turning(hub, robot, 90, 50)

    #Put back flip down to collect cylinders
    abs_backflip_turn(back_flipper, -105, 50, back_flipperInit)

    highspeed_pid(hub, robot, 31, 70, 90)
    abs_turning(hub, robot, 110, 40)

    '''
    back_flipper.run_for_degrees(60, 50)
    abs_turning(hub, robot, 180, 50)
    back_flipper.run_for_degrees(-10, 50)
    pid(hub, robot, 21, -50, 180)
    pid(hub, robot, 21, 50, 180)
    abs_turning(hub, robot, 195, 50)
    flipper.run_for_degrees(130, 90)
    abs_turning(hub, robot, 150, 50)
    '''

    #Code for HIGH FIVE and NRG collection and HYDRO DAM collection
    back_flipper.run_for_degrees(110, 50)
    abs_turning(hub, robot, 180, 40)
    pid(hub, robot, 5, 30, 180)
    back_flipper.run_for_degrees(-53, 50)

    #Do high five and collect units
    pid(hub, robot, 29, -30, 180)

    #Go forward
    pid(hub, robot, 18, 30, 180)

    #Turn 17 deg
    abs_turning(hub, robot, 197, 40)

    pid(hub, robot, 3, 30, 197)

    #Catch nrg
    flipper.run_for_degrees(70, 60)


    abs_turning(hub, robot, 170, 40)
    highspeed_pid(hub, robot, 55, 70, 170)

    #Turn 90 degrees clockwise
    abs_turning(hub, robot, 262, 40)
    back_flipper.run_for_degrees(110, 50)
    highspeed_pid(hub, robot, 36, 70, 262)
    back_flipper.run_for_degrees(-50, 50)
    highspeed_pid(hub, robot, 40, 90, 270)
    
def test():
    abs_flip_turn(flipper, 80, 50, flipperInit)

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

def highfive_collectnrg():
    abs_turning(hub, robot, 90, 40)

    #Do high five
    back_flipper.run_for_degrees(-60, 50)
    pid(hub, robot, 20, -30, 90)

    #Go forward
    pid(hub, robot, 14, 30, 90)

    #Turn 10 deg
    abs_turning(hub, robot, 107, 40)

    pid(hub, robot, 3, 30, 100)

    #Catch nrg
    flipper.run_for_degrees(70, 60)

    abs_turning(hub, robot, 90, 40)
    pid(hub, robot, 55, 50, 90)

    #Turn 90 degrees clockwise
    abs_turning(hub, robot, 180, 40)
    pid(hub, robot, 100, 50, 180)


#dino_run()
plat()
#highfive_collectnrg()
print(time.time()-startTime)
