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
        return correct - (curr - 360) + initAngle
    elif curr - correct < -180:
        return correct - (curr + 360) + initAngle
    else:
        return correct - curr + initAngle

def abs_turning(hub, robot, deg, speed):
    startTime = time.time()
    distOfWheels = 39
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
    flipper.set_stop_action('hold')

    back_flipper = Motor('A')
    return hub, robot, flipper, back_flipper

def waitUntilTap(hub):
    while True:
        if hub.motion_sensor.wait_for_new_gesture() == 'tapped':
            break

hub, robot, flipper, back_flipper = __init__()
initAngle = 0

def abs_flip_turn(flipper, correct, speed, flipperInit):
    flipper.run_to_position(flipperInit+correct, 'shortest path', speed)

flipperInit = flipper.get_position()

def mission():
    #Tv mission go forward and come back
    pid(hub, robot, 30, 50, 0)
    highspeed_pid(hub, robot, 44.5, -70, 0)
    abs_turning(hub, robot, -90, 50)

    abs_flip_turn(flipper, -20, 30, flipperInit)


    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    highspeed_pid(hub, robot, 60, 70, -90)
    hub.motion_sensor.reset_yaw_angle()
    abs_flip_turn(flipper, -90, 50, flipperInit)
    initAngle = -90
    #Turn right 35
    abs_turning(hub, robot, 45, 50)
    pid(hub, robot, 14, 30, 46)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 50)
    flipper.run_for_degrees(90, 50)
    flipper.run_for_degrees(-5, -50)

    #Go HOME
    highspeed_pid(hub, robot, 88.5, -70, 0)
    

    flipper.run_for_degrees(-85, 50)
    

    #Turn to do factory
    pid(hub, robot, 3, 30, 90)
    abs_turning(hub, robot, 42, 50)
    pid(hub, robot, 15, 50, 42)
    abs_turning(hub, robot, 0, 30)
    waitUntilTap(hub)

    #Dump + Grab
    highspeed_pid(hub, robot, 65, 70, 42)
    flipper.run_for_degrees(90, 50)
    highspeed_pid(hub, robot, 30, 70, 42)
    time.sleep(0.1)

    #Car
    abs_turning(hub, robot, 74, 30)
    flipper.run_for_degrees(-90, 50)
    time.sleep(0.5)
    flipper.run_for_degrees(90, 50)
    abs_turning(hub, robot, 45, 50)
    pid(hub, robot, 35, -50, 45)
    abs_turning(hub, robot, 137, 30)


    #Windmill
    #flipper.run_for_degrees(140, 30)
    pid(hub, robot, 10, 70, 137)
    fast_turning(hub, robot, 137, 45)
    for i in range(3):
        pid(hub, robot, 10.5, 30, 137)
        wait_for_seconds(0.5)
        pid(hub, robot, 9.75, -30, 137)
    
    pid(hub, robot, 5, -30, 137)

    #Put units on floor
    fast_turning(hub, robot, 0, 50)
    fast_turning(hub, robot, -45, 45)
    flipper.run_for_degrees(-90, 30)

    #Go home
    fast_turning(hub, robot, -105, 45)
    highspeed_pid(hub, robot, 55, 80, -115)

    #Align for dino
    abs_turning(hub, robot, 0, 45)

def test():
    flipper.run_for_degrees(-90, 50)

currentTime = time.time()
mission()
print(time.time()-currentTime)
