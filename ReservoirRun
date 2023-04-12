from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, robot, cm, speed, start_angle):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm)/0.0613888889
    print("Degrees needed: " + str(degrees_needed))
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle - hub.motion_sensor.get_yaw_angle())*GSPK
        print("Steer: " + str(steer))
        print("Degrees counted: " + str(motor.get_degrees_counted))
        #print(hub.motion_sensor.get_yaw_angle)
        robot.start(int(steer),speed)
        wait_for_seconds(0.01)
    robot.stop()

def turnDegrees(robot, deg):
    robot.move(round(deg*44/360, 2), 'cm', 100, 30)

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)

flipper = Motor('D')
back_flipper = Motor('A')

'''
DO NOT CHANGE
'''

#pid(hub, robot, 39, -30, 0)
print(hub.motion_sensor.get_yaw_angle())
turnDegrees(robot, -27)
print(hub.motion_sensor.get_yaw_angle())
#pid(hub, robot, 25, -30, 27)
