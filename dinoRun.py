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
    degrees_needed = abs(cm)/0.0613888889
    print(degrees_needed)
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
        wait_for_seconds(0.5)
    robot.stop()
    
hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)
robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
flipper.set_stop_action('hold')

'''
DO NOT CHANGE
'''
