from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
import time
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
def mission():
    pid(hub, robot, 54, 20, 0)
    flipper.run_for_degrees(90,-80)
    abs_turning(hub, robot, 5, 40)
    pid(hub, robot, 7, 30, 5)
    flipper.run_for_degrees(90,50)
    abs_turning(hub, robot, 0, 40)
    pid(hub, robot, 60, -30, 0)


currentTime = time.time()
mission()
print(time.time()-currentTime)
