'''
DO NOT CHANGE
'''

from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import math
import time

def pid(hub, robot, cm, speed, degrees_needed):
    motor = Motor('F')
    motor.set_degrees_counted(0)
    start_angle = hub.motion_sensor.get_yaw_angle()
    #Degrees needed per centimeter * centimers needed = degrees_needed
    #degrees_needed = abs(cm) * 360/22.1
    while abs(motor.get_degrees_counted())<=degrees_needed:
        GSPK = 1.7
        steer = (start_angle-hub.motion_sensor.get_yaw_angle())*GSPK
        robot.start(int(steer),speed)
        #wait_for_seconds(0.1)
    robot.stop()

def calDiff(curr, correct):
    if curr - correct > 180:
        return correct - (curr - 360)
    elif curr - correct < -180:
        return correct - (curr + 360)
    else:
        return correct - curr
        
def abs_turning(hub, robot, deg, speed):
    startTime = time.time()
    distOfWheels = 11 * math.pi
    calDiff(hub.motion_sensor.get_yaw_angle(), deg)
    robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, speed)
    for i in range(5):
        robot.move(distOfWheels*calDiff(hub.motion_sensor.get_yaw_angle(), deg)/360, 'cm', 100, 20)
        if calDiff(hub.motion_sensor.get_yaw_angle(), deg) == 0:
            break
    print('Total time is', time.time()-startTime)

def __init__():
    hub = PrimeHub()
    hub.motion_sensor.reset_yaw_angle()

    robot = MotorPair('F', 'B')
    robot.set_motor_rotation(22.1, 'cm')

    flipper = Motor('D')
    flipper.set_stop_action('hold')

    return hub, robot, flipper

hub, robot, flipper = __init__()
'''
DO NOT CHANGE
'''
