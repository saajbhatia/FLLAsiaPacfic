'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, tank, cm):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = cm/0.0613888889
    print(degrees_needed)
    while abs(motor.get_degrees_counted())<=degrees_needed: 
        print(motor.get_degrees_counted())
        GSPK = 1.4 
        steer = (0-hub.motion_sensor.get_yaw_angle())*GSPK 
        tank.start(int(steer),30)
    tank.stop()

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
flipper.set_stop_action('hold')

'''
DO NOT CHANGE
'''

flipper.run_for_degrees(90, 30)

#Go forward and do TV mission, then come back
pid(hub, robot, 23.5, 30)
pid(hub, robot, 19.75, -30)

#Turn to Car, Go to car, Do Car
turnDegrees(robot, -37)
pid(hub, robot, 79, 30)
flipper.run_for_degrees(-90, 30)

pid(hub, robot, 20, -30)
turnDegrees(robot, 79)
flipper.run_for_degrees(90, 30)
pid(hub,robot, 15, 30)

for i in range(3):
    pid(hub,robot, 10, 15)
    pid(hub,robot, 10, -15)
