from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
from math import *
def pid(hub, robot, cm, speed):
    motor = Motor('F')

    motor.set_degrees_counted(0)

    #Degrees needed per centimeter * centimers needed = degrees_needed
    degrees_needed = cm/0.0613888889
    print(degrees_needed)
    while abs(motor.get_degrees_counted())<=degrees_needed: 
        print(motor.get_degrees_counted())
        GSPK = 1.7 
        steer = (0-hub.motion_sensor.get_yaw_angle())*GSPK 
        robot.start(int(steer),speed)
        wait_for_seconds(0.5)
    print('CATS')
    robot.stop()

def turnDegrees(robot, deg):
    robot.move(round(44*deg/360, 2), 'cm', 100, 30)


hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)
robot.set_motor_rotation(22.1, 'cm')

flipper = Motor('D')
flipper.set_stop_action('hold')
#flipper.run_for_degrees(161, 30)

pid(hub, robot, 200)

raise SystemError

robot.move(20,'cm', 0, 30)
robot.move(-17,'cm', 0, 30)
turnDegrees(robot, -38)

hub.motion_sensor.reset_yaw_angle()

robot.move(74,'cm', 0, 30)

'''
flipper.start(0)
for i in range(3):
    motor_pair.move(10, 'cm', 0, 30)
    motor_pair.move(-10, 'cm', 0, 30)
print('second wait')
flipper.stop()
'''
