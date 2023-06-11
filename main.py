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

def abs_turning(hub, robot, deg, speed, offset = 3):
    startTime = time.time()
    distOfWheels = 39.0
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) <= offset:
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

def abs_flip_turn(flipper, correct, speed, flipperInit):
    flipper.run_to_position(calDiffFlip(flipperInit+correct), 'shortest path', speed)

def abs_backflip_turn(back_flipper, correct, speed, back_flipperInit):
    back_flipper.run_to_position(calDiffFlip(back_flipperInit+correct), 'shortest path', speed)


def powerplant_and_tv(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    #Go to pwr plant
    abs_flip_turn(flipper, 63, 50, flipperInit)
    highspeed_pid(hub, robot, 50, 80, 0)
    abs_turning(hub, robot, -6, 50, 0)
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
    abs_flip_turn(flipper, 0, 50, flipperInit)

def windmill(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    hub.motion_sensor.reset_yaw_angle()

    abs_turning(hub, robot, -46, 30)

    #Dump + Grab
    highspeed_pid(hub, robot, 60, 80, -46)
    abs_flip_turn(flipper, 90, 30, flipperInit)
    pid(hub, robot, 7, -50, -46)

    #Turn for windmill
    fast_turning(hub, robot, 60, 30)
    wait_for_seconds(0.1)
    abs_turning(hub, robot, 45, 30)

    #Do Windmill
    highspeed_pid(hub, robot, 32, 80, 45)
    pid(hub, robot, 5.5, -50, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 5.5, 30, 45)
    pid(hub, robot, 6, -50, 45)
    wait_for_seconds(0.3)
    pid(hub, robot, 6.5, 30, 45)
    pid(hub, robot, 13, -50, 45)

    #Put units on floor
    left_abs_turning(hub, robot, -145, 50)
    abs_turning(hub, robot, -145, 50)
    wait_for_seconds(1)
    flipper.run_for_degrees(-90, 30)

    #Go home
    fast_turning(hub, robot, -215, 45)
    highspeed_pid(hub, robot, 57, 100, -205)
    abs_flip_turn(flipper, 0, 50, flipperInit)

def dino_only_collect_water_run(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    hub.motion_sensor.reset_yaw_angle()
    highspeed_pid(hub, robot, 130, 100, 0)
    fast_turning(hub, robot, 37, 50)
    flipper.run_for_degrees(85, 60)
    fast_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 40, 100, 0)
    back_flipper.set_stop_action('coast')
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)
    abs_flip_turn(flipper, 0, 30, flipperInit)

def plat(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    #Go to platform
    abs_flip_turn(flipper, 90, 50, flipperInit)
    back_flipper.set_stop_action('hold')
    abs_backflip_turn(back_flipper, 69, 30, back_flipperInit)
    highspeed_pid(hub, robot, 54.5, 70, 0)
    back_flipper.set_stop_action('coast')
    abs_backflip_turn(back_flipper, 48, 30, back_flipperInit)
    abs_backflip_turn(back_flipper, 69, 30, back_flipperInit)
    #pid(hub, robot, 0.25, 30, 0)
    abs_turning(hub, robot, -6, 30, 0)

    #Flip/do platform
    for i in range(3):
        abs_flip_turn(flipper, 50, 90, flipperInit)
        abs_flip_turn(flipper, 80, 90, flipperInit)

    #Go to solar farm energy units
    abs_turning(hub, robot, 39, 50)
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)


    abs_flip_turn(flipper, 0, 50, flipperInit)
    highspeed_pid(hub, robot, 28, 80, 39)
    abs_turning(hub, robot, 90, 50)

    #Put back flip down to collect cylinders
    abs_backflip_turn(back_flipper, -115, 50, back_flipperInit)

    highspeed_pid(hub, robot, 32, 80, 90)

    #Code for HIGH FIVE and NRG collection and HYDRO DAM collection
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)
    pid(hub, robot, 4, -40, 90)
    abs_turning(hub, robot, 180, 40)
    #pid(hub, robot, 1, 30, 180)
    back_flipper.set_stop_action('hold')
    abs_backflip_turn(back_flipper, -73, 30, back_flipperInit)

    #Do high five and collect units
    highspeed_pid(hub, robot, 24, -30, 180)
    back_flipper.set_stop_action('coast')
    abs_backflip_turn(back_flipper, -65, 30, back_flipperInit)


    #Go forward
    highspeed_pid(hub, robot, 20, 30, 180)
    abs_turning(hub, robot, 195, 40)
    pid(hub, robot, 1, 30, 195)

    #Catch nrg
    flipper.run_for_degrees(70, 60)
    abs_turning(hub, robot, 165, 40)
    highspeed_pid(hub, robot, 51, 80, 165)

    #Turn 90 degrees clockwise
    abs_turning(hub, robot, 260, 40)
    #abs_backflip_turn(back_flipper, -10, 30, back_flipperInit)
    pid(hub, robot, 85, 90, 260)

    abs_flip_turn(flipper, 0, 55, flipperInit)
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)

def dump(hub, robot, flipper, flipperInit):
    #abs_turning(hub, robot, 3, 30)
    highspeed_pid(hub, robot, 74, 80, 3)
    flipper.run_for_rotations(0.15, 50)
    flipper.run_for_rotations(0.15, -50)
    pid(hub, robot, 3, -40, 0)
    abs_flip_turn(flipper, 90, 50, flipperInit)
    pid(hub, robot, 69, -80, 10)
    abs_flip_turn(flipper, 0, 50, flipperInit)

def dumpAndTruck(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    dump(hub, robot, flipper, flipperInit)
    abs_turning(hub, robot, -45, 30, 0)
    #used to be 29.5
    pid(hub, robot, 28, 50, -45)
    abs_turning(hub, robot, 0, 30, 0)
    abs_flip_turn(flipper, 90, 50, flipperInit)
    #abs_turning(hub, robot, 0, 30)
    pid(hub, robot, 8, 40, 0)
    abs_flip_turn(flipper, 95, 50, flipperInit)
    pid(hub, robot, 5, 20, 0)
    wait_for_seconds(0.5)
    #abs_turning(hub, robot, 3, 30, 0)
    pid(hub, robot, 5, -10, 0)
    highspeed_pid(hub, robot, 35, -85, 0)
    #abs_flip_turn(flipper, 0, 50, flipperInit)

def reservoir(hub, robot, flipper, flipperInit):
    highspeed_pid(hub, robot, 15, 70, 0)
    abs_turning(hub, robot, 45, 30)
    return
    highspeed_pid(hub, robot, 75, 70, 45)

    abs_turning(hub, robot, 90, 30)
    highspeed_pid(hub, robot, 20, 70, 90)
    abs_turning(hub, robot, 140, 30)

    pid(hub, robot, 25, 40, 140)
    pid(hub, robot, 15.5, -30, 140)

    diff = 140 - hub.motion_sensor.get_yaw_angle()
    hub.motion_sensor.reset_yaw_angle()
    abs_turning(hub, robot, 93 + diff, 30, 0)

    pid(hub, robot, 9, 40, 93 + diff)

    flipper.run_for_degrees(65, 10)


    pid(hub, robot, 10, -50, 93 + diff)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    return
    flipper.run_for_degrees(60, -10)
    pid(hub, robot, 10, -40, 93 + diff)

#THIS CODE DOES THE ORIGINAL RES STUFF BUT ALSO THE CAR
#Latest 6/6
def reservoir2(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    hub.motion_sensor.reset_yaw_angle()
    #highspeed_pid(hub, robot, 15, 70, 0)

    #Turn and go parallel to hydro dam
    abs_turning(hub, robot, -45, 30)
    highspeed_pid(hub, robot, 75, 70, -45)

    #Turn to 0 and put innovation thing
    abs_turning(hub, robot, 0, 30)
    highspeed_pid(hub, robot, 20, 70, 0)
    abs_turning(hub, robot, 50, 30)

    pid(hub, robot, 18, 40, 50)
    pid(hub, robot, 11, -30, 50)

    diff = 50 - hub.motion_sensor.get_yaw_angle()
    hub.motion_sensor.reset_yaw_angle()

    #Hydro units
    abs_turning(hub, robot, 93 + diff, 30, 0)

    #used to be 1.35
    pid(hub, robot, 2, 40, 93 + diff)

    flipper.run_for_degrees(65, 10)


    pid(hub, robot, 11, -50, 93 + diff)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    pid(hub, robot, 8, -40, 93 + diff)
    abs_turning(hub, robot, 135, 30)

    #abs_backflip_turn(back_flipper, -65, 30, back_flipperInit)
    back_flipper.run_for_degrees(75, 60)

    #Go to car and flip
    pid(hub, robot, 16.5, -40, 135 + diff)
    back_flipper.run_for_degrees(-30, 60)

    #flipper back down
    back_flipper.run_for_degrees(30, 60)
    pid(hub, robot, 3, 40, 135 + diff)

    #Up and go to truck
    back_flipper.run_for_degrees(-70, 60)
    pid(hub, robot, 30, -40, 135 + diff)

    #Turn and truck
    abs_turning(hub, robot, 85, 30)
    pid(hub, robot, 23, -60, 85 + diff)

def test(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    back_flipper.run_for_degrees(-70, 60)
#please do not have any code outside of functions
def mainmenu():

    #Create variables
    currentTime = time.time()
    hub, robot, flipper, back_flipper = __init__()

    waitUntilTap(hub)
    print("New Run:")

    #Run 1 - TV Run - Missions Done: Powerplant, TV, Car, Windmill, Toy Factory
    powerplant_and_tv(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Car: " + str(time.time()-transistion))
    windmill(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))

    #Run 2 - Dino Run - Missions Done: Bring Dino Home, Get Water Unit
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Dino: " + str(time.time()-transistion))
    dino_only_collect_water_run(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))

    #Run 3 - Platform Run - Missions Done: Hydro Dam, Oil Platform, Dump Units into White Box, Solar Farm, Highfive, Collect Water Units
    transistion = time.time()
    waitUntilTap(hub)
    hub.motion_sensor.reset_yaw_angle()
    print("Transition Time for Platform: " + str(time.time()-transistion))
    plat(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Dump: " + str(time.time()-transistion))
    #Reset everything for dump
    hub, robot, flipper, back_flipper = __init__()

    dumpAndTruck(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))

    #Run 4 - Reservoir Run - Missions Done: Hook up water units, drop off innovation model
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Reservoir: " + str(time.time()-transistion))
    reservoir2(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print(time.time()-currentTime)


#Change this to run the thing you want to run.
def Run():
    currentTime = time.time()
    hub, robot, flipper, back_flipper = __init__()
    #plat(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    #reservoir2(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    dumpAndTruck(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print(time.time()-currentTime)

mainmenu()
