from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair

from spike.control import wait_for_seconds, wait_until, Timer

from math import *

def pid(hub, robot, cm, speed):

    motor = Motor('F')
    motor.set_degrees_counted(0)

    start_angle = hub.motion_sensor.get_yaw_angle()

    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = abs(cm)/0.0613888889

    print(degrees_needed)

    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7

        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK

        robot.start(int(steer),speed)

        wait_for_seconds(0.5)

    robot.stop()

    
def turnDegrees(robot, deg):

    robot.move(round(44*deg/360, 2), 'cm', 100, 30)


hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)
robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
flipper.set_stop_action('hold')
flipper.run_for_degrees(161, 30)

#Go forward and do TV mission, then come back
pid(hub, robot, 16.25, 30)
pid(hub, robot, 15.5, -30)

#Turn to Car, Go to car
turnDegrees(robot, -46)
pid(hub, robot, 79, 30)



