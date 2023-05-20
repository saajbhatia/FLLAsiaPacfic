#Importing libraries
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time

#Setting up PID controller
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

#Used for absolute turning
def calDiff(curr, correct):
    if curr - correct > 180:
        return correct - (curr - 360)
    elif curr - correct < -180:
        return correct - (curr + 360)
    else:
        return correct - curr

#Setting up absolute turning function
def abs_turning(hub, robot, deg, speed):
    distOfWheels = 38.25
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
    #print("Robot is off by " + str(hub.motion_sensor.get_yaw_angle() - deg) + " degrees.")

#Setting up the motors
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

#Used for counting how long the program took to run
startTime = time.time()

print(" ")
print(" ")
print(" ")
print(" ")
print(" ")



def reservoir1():
    highspeed_pid(hub, robot, 15, 70, 0)
    abs_turning(hub, robot, 45, 30)
    highspeed_pid(hub, robot, 75, 70, 45)

    abs_turning(hub, robot, 90, 30)
    highspeed_pid(hub, robot, 20, 70, 90)
    abs_turning(hub, robot, 140, 30)
    
    pid(hub, robot, 25, 40, 140)
    pid(hub, robot, 17.5, -30, 140)

    print(hub.motion_sensor.get_yaw_angle())

    diff = 140 - hub.motion_sensor.get_yaw_angle()
    hub.motion_sensor.reset_yaw_angle()
    abs_turning(hub, robot, 93 + diff, 30)

    print(hub.motion_sensor.get_yaw_angle())

    pid(hub, robot, 8, 30, 93 + diff)
    abs_turning(hub, robot, 93 + diff, 50)
    flipper.run_for_degrees(40, 10)
    pid(hub, robot, 5, -40, 93 + diff)
    flipper.run_for_degrees(40, -10)
    pid(hub, robot, 10, -40, 93 + diff)

def reservoir2():
    hub.motion_sensor.reset_yaw_angle()
    abs_turning(hub, robot, 40, 30)
    highspeed_pid(hub, robot, 45, -70, 40)
    abs_turning(hub, robot, -5, 30)
    pid(hub, robot, 22, -40, -5)


reservoir1()
reservoir2()

print("Total time: " + str(time.time() - startTime) + " seconds")
