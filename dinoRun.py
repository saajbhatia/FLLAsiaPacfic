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
    
    flipper.run_for_degrees(18,-50)

    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    pid(hub, robot, 66, 30, 0)
    flipper.run_for_degrees(80,-90)

    #Turn right 35
    abs_turning(hub, robot, 35, 10)
    pid(hub, robot, 15, 15, 34)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 10)
    flipper.run_for_degrees(90, 90)
    flipper.run_for_degrees(-5, -75)
    #Go HOME 
    pid(hub, robot, 84, -60, 0)
    abs_turning(hub, robot, 90, 0)

    #Tv mission go forward and come back
    pid(hub, robot, 39, 40, 90)
    pid(hub, robot, 35, -40, 90)
    
    #Turn to do factory
    abs_turning(hub, robot, 45, 60)
    
    #Setup
    flipper.run_for_degrees(-140, 30)

    #Dump + Grab
    pid(hub, robot, 75, 50, 45)
    flipper.run_for_degrees(140, 30)
    pid(hub, robot, 21, 50, 45)

    #Car
    abs_turning(hub, robot, 69, 45)
    flipper.run_for_degrees(-140, 30)
    abs_turning(hub, robot, 45, 45)
    pid(hub, robot, 28, -45, 45)
    abs_turning(hub, robot, 137, 45)

    #Windmill
    flipper.run_for_degrees(140, 30)
    pid(hub, robot, 15, 50, 137)
    abs_turning(hub, robot, 137, 45)
    for i in range(3):
        pid(hub, robot, 11.5, 30, 137)
        pid(hub, robot, 10.5, -30, 137)
        wait_for_seconds(0.5)

    #Put units on floor
    abs_turning(hub, robot, -44, 45)
    flipper.run_for_degrees(-140, 30)

    #Go home
    abs_turning(hub, robot, -105, 45)
    pid(hub, robot, 55, 50, -115)

    #Align for dino
    abs_turning(hub, robot, 0, 45)

currentTime = time.time()
mission()
print(time.time()-currentTime)
