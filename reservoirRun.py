from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *

def pid(hub, robot, cm, speed, start_angle):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    #start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm) * 360/22.1
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
        #wait_for_seconds(0.1)
    robot.stop()

def abs_turning(hub, robot, deg, speed):
    for i in range(5):
        robot.move(33*(deg-hub.motion_sensor.get_yaw_angle())/360, 'cm', 100, speed)

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)

flipper = Motor('D')
back_flipper = Motor('A')

'''
DO NOT CHANGE
'''

print(" ")
print(" ")
print(" ")
print(" ")
print(" ")



print("Start Angle: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be 0")
print(" ")

pid(hub, robot, 39, -20, 0)
print("Angle before first turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be 0")
print(" ")

abs_turning(hub, robot, -50, 5)
print("Angle after first turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -50")
print(" ")

pid(hub, robot, 53, -20, -50)
print("Angle before second turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -50")
print(" ")

abs_turning(hub, robot, -15, 5)
print("Angle after second turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -15")
print(" ")

pid(hub, robot, 13, 20, -15)
print("Angle before third turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -15")
print(" ")

abs_turning(hub, robot, -33, 5)
print("Angle after third turn: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -33")
print(" ")

pid(hub, robot, 2, 20, -33)
print("Angle before flick down: " + str(hub.motion_sensor.get_yaw_angle()))
print("Should be -33")
print(" ")

wait_for_seconds(1)

back_flipper.run_for_degrees(90, -60)
