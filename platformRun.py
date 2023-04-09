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
'''
DO NOT CHANGE
'''

flipper.run_for_degrees(90, 30)
robot.move(round(33*30/360, 2), 'cm', 100, 15)
pid(hub, robot, 0.25, 30)
robot.move(round(33*-30/360, 2), 'cm', 100, 15)
pid(hub, robot, 42, 30)
