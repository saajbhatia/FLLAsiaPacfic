'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import math
import time

def pid(hub, robot, cm, speed):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/17.8
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
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
    distOfWheels = 37
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
    print('Total time is', time.time()-startTime)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(17.5, 'cm')

    flipper = Motor('D')
    flipper.set_stop_action('hold')

    return hub, robot, flipper

hub, robot, flipper = __init__()
'''
DO NOT CHANGE
'''

def tv():
    flipper.run_for_degrees(90, 30)

    #Go forward and do TV mission, then come back
    pid(hub, robot, 23.5, 30)
    pid(hub, robot, 19.75, -30)

def car():
    #Turn to Car, Go to car, Do Car
    turnDegrees(robot, -37)
    pid(hub, robot, 79, 30)
    flipper.run_for_degrees(-90, 30)

    pid(hub, robot, 20, -30)
    turnDegrees(robot, 79)
    flipper.run_for_degrees(90, 30)
    pid(hub,robot, 15, 30)

def windmill():
    for i in range(3):
        pid(hub,robot, 10, 15)
        pid(hub,robot, 10, -15)

def check_angle(ang):
    if hub.motion_sensor.get_yaw_angle() >= ang:
        return

def mainMissions():
    #flipper.run_for_degrees(90, 30)
    pid(hub, robot, 31, 30, 0)
    pid(hub, robot, 35, -30, 0)
    jyrt
    abs_turning(hub, robot, -47, 20)
    pid(hub, robot, 93, 30, -47)
    return
    abs_turning(hub, robot, -10, 10)
    flipper.run_for_degrees(-120, 15)
    abs_turning(hub, robot, -47, 20)
    pid(hub, robot, 30, -30)
    abs_turning(hub, robot, 45, 20)
    flipper.run_for_degrees(120, 15)
    pid(hub, robot, 25, 30)
    for i in range(3):
        pid(hub, robot, 7, 20)
        pid(hub, robot, 7, -20)
        wait_for_seconds(0.5)
    abs_turning(hub, robot, 155, 20)
    pid(hub, robot, 60, 80)

startTime = time.time()
pid(hub, robot, 30, 40)
print(time.time()-startTime)
