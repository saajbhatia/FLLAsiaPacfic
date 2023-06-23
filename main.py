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
    back_flipper.set_stop_action('brake')
    back_flipper.set_degrees_counted(0)
    return hub, robot, flipper, back_flipper

def waitUntilTap(hub):
    hub.right_button.wait_until_pressed()

def abs_flip_turn(flipper, correct, speed, flipperInit):
    flipper.run_to_position(correct, 'shortest path', speed)

def abs_backflip_turn(back_flipper, correct, speed, back_flipperInit):
    back_flipper.run_to_position(correct, 'shortest path', speed)

def no_diff_abs_backflip_turn(back_flipper, correct, speed, back_flipperInit):
    back_flipper.run_to_position(correct, 'shortest path', speed)


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

    abs_turning(hub, robot, -47, 30)

    #Dump + Grab
    highspeed_pid(hub, robot, 65, 80, -47)
    abs_flip_turn(flipper, 90, 30, flipperInit)
    highspeed_pid(hub, robot, 18, 80, -47)

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
    pid(hub, robot, 5, -20, 45)
    wait_for_seconds(0.5)
    pid(hub, robot, 6, 50, 45)
    pid(hub, robot, 5, -20, 45)
    wait_for_seconds(0.5)
    pid(hub, robot, 6, 50, 45)
    pid(hub, robot, 18, -20, 45)

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
    pid(hub, robot, 4, -50, -46)

    #Turn for windmill
    fast_turning(hub, robot, 60, 30)
    wait_for_seconds(0.1)
    abs_turning(hub, robot, 45, 30)

    #Do Windmill
    highspeed_pid(hub, robot, 28, 80, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 6.5, -15, 45)
    wait_for_seconds(0.2)
    pid(hub, robot, 6, 50, 45)
    wait_for_seconds(0.1)
    pid(hub, robot, 6, -15, 45)
    wait_for_seconds(0.5)
    pid(hub, robot, 8, 50, 45)
    wait_for_seconds(0.2)
    pid(hub, robot, 15, -50, 45)

    #Put units on floor
    left_abs_turning(hub, robot, -145, 50)
    abs_turning(hub, robot, -145, 50)
    wait_for_seconds(1)
    flipper.run_for_degrees(-90, 30)

    #Go home
    fast_turning(hub, robot, -215, 45)
    highspeed_pid(hub, robot, 60, 100, -205)
    abs_flip_turn(flipper, 0, 50, flipperInit)

def dino_only_collect_water_run(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    hub.motion_sensor.reset_yaw_angle()
    highspeed_pid(hub, robot, 130, 100, 0)
    fast_turning(hub, robot, 37, 50)
    flipper.run_for_degrees(85, 60)
    fast_turning(hub, robot, 0, 50)
    highspeed_pid(hub, robot, 40, 100, 0)
    #back_flipper.set_stop_action('coast')
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)
    abs_flip_turn(flipper, 0, 30, flipperInit)

def plat(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    #Go to platform
    back_flipper.set_stop_action('hold')
    flipper.set_stop_action('brake')
    abs_flip_turn(flipper, 90, 50, flipperInit)
    abs_backflip_turn(back_flipper, 80, 30, back_flipperInit)
    highspeed_pid(hub, robot, 55, 70, 0)
    abs_turning(hub, robot, -7, 30, 0)

    #Put back flip up, do hydro dam
    abs_backflip_turn(back_flipper, 65, 50, back_flipperInit)
    abs_backflip_turn(back_flipper, 80, 30, back_flipperInit)


    #Flip/do platform
    for i in range(3):
        abs_flip_turn(flipper, 50, 90, flipperInit)
        abs_flip_turn(flipper, 90, 90, flipperInit)


    #Go to solar farm energy units
    abs_turning(hub, robot, 39, 50)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    abs_backflip_turn(back_flipper, 0, 50, back_flipperInit)
    highspeed_pid(hub, robot, 30.5, 80, 39)
    abs_turning(hub, robot, 90, 50)
    pid(hub, robot, 1, -30, 90)

    #Put back flip down to collect cylinders
    back_flipper.run_to_position(255, "counterclockwise", 30)
    highspeed_pid(hub, robot, 33, 80, 90)

    #Code for HIGH FIVE and NRG collection and HYDRO DAM collection
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)
    pid(hub, robot, 4.5, -40, 90)
    abs_turning(hub, robot, 180, 40)
    back_flipper.set_degrees_counted(0)
    abs_backflip_turn(back_flipper, 300, 30, back_flipperInit)

    #Do high five and collect units
    highspeed_pid(hub, robot, 24, -30, 180)
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)

    #Go forward
    highspeed_pid(hub, robot, 21, 30, 180)
    abs_turning(hub, robot, 200, 40)

    #Catch nrg
    abs_flip_turn(flipper, 80, 50, flipperInit)
    abs_turning(hub, robot, 165, 50)
    highspeed_pid(hub, robot, 49, 80, 165)

    #Go home
    abs_turning(hub, robot, 270, 40)
    pid(hub, robot, 80, 100, 270)
    abs_flip_turn(flipper, 0, 55, flipperInit)
    abs_backflip_turn(back_flipper, 0, 30, back_flipperInit)

def dump(hub, robot, flipper, flipperInit):
    #abs_turning(hub, robot, 3, 30)
    highspeed_pid(hub, robot, 74.25, 80, 3)
    flipper.run_for_rotations(0.15, 50)
    flipper.run_for_rotations(0.15, -50)
    pid(hub, robot, 3, -40, 0)
    abs_flip_turn(flipper, 90, 50, flipperInit)
    pid(hub, robot, 69, -80, 10)
    abs_flip_turn(flipper, 0, 50, flipperInit)

def dumpAndTruck(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    dump(hub, robot, flipper, flipperInit)
    flipper.set_stop_action("hold")
    abs_turning(hub, robot, -45, 30, 0)
    #used to be 29.5
    pid(hub, robot, 21, 30, -45)
    abs_turning(hub, robot, 0, 30, 0)
    abs_flip_turn(flipper, 98, 50, flipperInit)
    #abs_turning(hub, robot, 0, 30)
    pid(hub, robot, 12, 40, 0)
    pid(hub, robot, 6, 20, 0)
    wait_for_seconds(0.5)
    flipper.set_stop_action("coast")
    abs_flip_turn(flipper, 87, 50, flipperInit)
    #abs_turning(hub, robot, 3, 30, 0)
    pid(hub, robot, 5, -10, 0)
    highspeed_pid(hub, robot, 36, -100, 0)
    #abs_flip_turn(flipper, 0, 50, flipperInit)


#THIS CODE DOES THE ORIGINAL RES STUFF BUT ALSO THE CAR
#Latest 6/6
def reservoir2(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    #hub.motion_sensor.reset_yaw_angle()

    #Turn and go parallel to hydro dam
    abs_turning(hub, robot, -45, 30)
    highspeed_pid(hub, robot, 75, 80, -45)

    #Turn to 0 and put innovation thing
    abs_turning(hub, robot, 0, 30)
    highspeed_pid(hub, robot, 20, 80, 0)
    abs_turning(hub, robot, 50, 30)

    highspeed_pid(hub, robot, 18, 80, 50)
    highspeed_pid(hub, robot, 12, -40, 50)

    diff = 50 - hub.motion_sensor.get_yaw_angle()
    print ("diff is" + str(diff))
    hub.motion_sensor.reset_yaw_angle()

    #Turn and place hydro units
    abs_turning(hub, robot, 90 + diff, 30, 0)
    pid(hub, robot, 4.5, 10, 90 + diff)
    flipper.run_for_degrees(50, 10)

    #Go back to car and turn
    #NOTE: this may need to be shortened by a cm if the backflipper gets stuck
    pid(hub, robot, 5, -30, 90 + diff)
    abs_turning(hub, robot, 115, 30)
    abs_flip_turn(flipper, 0, 50, flipperInit)
    #Go to car and place flip down
    highspeed_pid(hub, robot, 18.5, -50, 115 + diff)
    abs_backflip_turn(back_flipper, 115, 30, back_flipperInit)

    #Go to car and do car
    highspeed_pid(hub, robot, 14, -50, 115 + diff)
    abs_backflip_turn(back_flipper, 50, 80, back_flipperInit)
    abs_backflip_turn(back_flipper, 110, 80, back_flipperInit)

    #Come back from car
    pid(hub, robot, 5.5, 30, 115+diff)
    abs_backflip_turn(back_flipper, 0, 80, back_flipperInit)
    abs_turning(hub, robot, 135, 30)
    #Go to truck
    highspeed_pid(hub, robot, 25, -70, 135 + diff)
    #Turn and truck
    abs_turning(hub, robot, 77, 50)
    highspeed_pid(hub, robot, 23, -70, 77 + diff)
    return



#please do not have any code outside of functions
def mainmenu():

    #Create variables
    currentTime = time.time()
    hub, robot, flipper, back_flipper = __init__()

    waitUntilTap(hub)
    print("New Run:")
    hub, robot, flipper, back_flipper = __init__()
    #Run 1 - TV Run - Missions Done: Powerplant, TV, Car, Windmill, Toy Factory
    powerplant_and_tv(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Car: " + str(time.time()-transistion))
    hub, robot, flipper, back_flipper = __init__()
    windmill(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))

    #Run 2 - Dino Run - Missions Done: Bring Dino Home, Get Water Unit
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Dino: " + str(time.time()-transistion))
    hub, robot, flipper, back_flipper = __init__()
    dino_only_collect_water_run(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))

    #Run 3 - Platform Run - Missions Done: Hydro Dam, Oil Platform, Dump Units into White Box, Solar Farm, Highfive, Collect Water Units
    transistion = time.time()
    waitUntilTap(hub)
    hub.motion_sensor.reset_yaw_angle()
    print("Transition Time for Platform: " + str(time.time()-transistion))
    hub, robot, flipper, back_flipper = __init__()
    plat(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print("Run time for Platform: " + str(time.time()-transistion))

    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Dump: " + str(time.time()-transistion))
    #Reset everything for dump
    hub, robot, flipper, back_flipper = __init__()
    dumpAndTruck(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print("Run time for Dump: " + str(time.time()-transistion))
    #Run 4 - Reservoir Run - Missions Done: Hook up water units, drop off innovation model
    transistion = time.time()
    waitUntilTap(hub)
    print("Transition Time for Reservoir: " + str(time.time()-transistion))
    hub, robot, flipper, back_flipper = __init__()
    reservoir2(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print(time.time()-currentTime)


def test(hub, robot, flipper, flipperInit, back_flipper, back_flipperInit):
    for x in range(10):
        powerplant_and_tv(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
        waitUntilTap(hub)

#Change this to run the thing you want to run.
def Run():
    currentTime = time.time()
    hub, robot, flipper, back_flipper = __init__()
    test(hub, robot, flipper, int(flipper.get_position()), back_flipper, int(back_flipper.get_position()))
    print(time.time()-currentTime)

#Run()
mainmenu()
quit()
