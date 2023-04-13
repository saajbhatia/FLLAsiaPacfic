'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, robot, cm, speed, start_angle):
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
back_flipper = Motor('A')


'''
DO NOT CHANGE
'''

#robot.move(round(44*30/360, 2), 'cm', 100, 15)
flipper.run_for_degrees(160, 90)
pid(hub, robot, 50, 30, 0)
#flipping platforn run
for i in range(3):
    flipper.run_for_degrees(-40, 80)
    flipper.run_for_degrees(40, 80)
    back_flipper.run_for_degrees(70, 80)
    back_flipper.run_for_degrees(-70, 80)
abs_turning(hub, robot, 37, 15)
pid(hub, robot, 40, 30, 37)
abs_turning(hub, robot, 90, 15)
#back flip goes down to collect cubes
back_flipper.run_for_degrees(-120, 50)
pid(hub, robot, 37, 30, 90)
