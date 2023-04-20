#Importing libraries
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time

#Setting up PID controller
def pid(hub, robot, cm, speed, start_angle):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    degrees_needed = abs(cm) * 360/22.1
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
    print("Robot is off by " + str(hub.motion_sensor.get_yaw_angle() - start_angle) + " degrees.")
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
    print("Robot is off by " + str(hub.motion_sensor.get_yaw_angle() - deg) + " degrees.")

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

'''
DO NOT CHANGE
'''
#Start
pid(hub, robot, 70, -30, 0)

abs_turning(hub, robot, 45, 50)

pid(hub, robot, 50, -30, 45)

abs_turning(hub, robot, 138, 30)

pid(hub, robot, 13.5, 50, 138)

back_flipper.run_for_degrees(90, -60)

#End
print("Time took to run program was: " + str(time.time()-startTime) + " seconds.")
