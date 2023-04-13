'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, robot, cm, speed):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/22.1
    print(degrees_needed)
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
        wait_for_seconds(0.1)
    robot.stop()

def abs_turning(hub, robot, deg, speed):
    for i in range(5):
        robot.move(33*(deg-hub.motion_sensor.get_yaw_angle())/360, 'cm', 100, speed)

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
flipper.set_stop_action('hold')

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

def testing():
    flipper.run_for_degrees(95, 30)
    pid(hub, robot, 21, 30)
    pid(hub, robot, 26, -30)
    abs_turning(hub, robot, -47, 15)
    pid(hub, robot, 93, 30)
    abs_turning(hub, robot, -10, 10)
    flipper.run_for_degrees(-95, 30)
testing()
