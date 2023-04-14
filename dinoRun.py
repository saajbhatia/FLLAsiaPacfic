from spike import PrimeHub, LightMatrix, Button, StatusLight, ForceSensor, MotionSensor, Speaker, ColorSensor, App, DistanceSensor, Motor, MotorPair
from spike.control import wait_for_seconds, wait_until, Timer
import math
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
        wait_for_seconds(0.01)
    robot.stop()

def abs_turning(hub, robot, deg, speed):
    startTime = time.time()
    distOfWheels = 11 * math.pi
    robot.move(distOfWheels*(deg-hub.motion_sensor.get_yaw_angle())/360, 'cm', 100, speed)
    for i in range(5):
        robot.move(distOfWheels*(deg-hub.motion_sensor.get_yaw_angle())/360, 'cm', 100, 20)
        if deg-hub.motion_sensor.get_yaw_angle() == 0:
            break
    print('Total time is', time.time()-startTime)

hub = PrimeHub()
hub.motion_sensor.reset_yaw_angle()

robot = MotorPair('F', 'B')
robot.set_default_speed(50)

flipper = Motor('D')
back_flipper = Motor('A')
def mission():
    pid(hub, robot, 54, 50, 0)
    flipper.run_for_degrees(90,-90)
    abs_turning(hub, robot, 5, 40)
    pid(hub, robot, 8, 30, 5)
    flipper.run_for_degrees(90,20)
    abs_turning(hub, robot, 0, 40)
    pid(hub, robot, 63, -50, 0)


currentTime = time.time()
mission()
print(time.time()-currentTime)
