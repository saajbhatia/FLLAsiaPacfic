'''
DO NOT CHANGE
'''
from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, robot, cm, speed, start_angle):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm)/0.0613888889
    print(degrees_needed)
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        print(steer)
        print(hub.motion_sensor.get_yaw_angle)
        robot.start(int(steer),speed)
        wait_for_seconds(0.01)
    robot.stop()

def turnDegrees(robot, deg):
    robot.move(round(44*deg/360, 2), 'cm', 100, 30)

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)
#robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
#flipper.set_stop_action('hold')
back_flipper = Motor('A')
#back_flipper.set_stop_action('hold')

'''
DO NOT CHANGE
'''


#robot.move(round(44*30/360, 2), 'cm', 100, 15)
flipper.run_for_degrees(165, 90)
pid(hub, robot, 50, 40, 0)
for i in range(3):
    flipper.run_for_degrees(-30, 60)
    flipper.run_for_degrees(30, 60)
    #back_flipper.run_for_degrees(70, 50)
    #back_flipper.run_for_degrees(-70, 50)
