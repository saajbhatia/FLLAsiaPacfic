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
    
    #Go to pwr plant OG val 53.75 then 52.5 then 47 and lift bar
    pid(hub, robot, 53, 30, 0)
    flipper.run_for_degrees(80,-90)

    #Turn right 35
    abs_turning(hub, robot, 34, 10)
    pid(hub, robot, 13, 15, 34)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 10)
    flipper.run_for_degrees(90, 90)

    #Go HOME 
    pid(hub, robot, 75, -30, 0)
    abs_turning(hub, robot, 45, 10)
    
    '''
    abs_turning(hub, robot, -8, 20)
    flipper.run_for_degrees(90,-90)

    #Turn back after lifting pwr plant thingy
    abs_turning(hub, robot, 0, 20)
    
    #Go forward and push down pwr plant thingy
    pid(hub, robot, 8, 20, 0)
    flipper.run_for_degrees(110,20)
    #abs_turning(hub, robot, 0, 40)

    #Go back and turn 45deg
    #abs_turning(hub, robot, 20, 0)
    print(hub.motion_sensor.get_yaw_angle())
    print("Is that vegan?")
    pid(hub, robot, 63, -20, 0)
    abs_turning(hub, robot, 0, 20)
    return
    
    abs_turning(hub, robot, 20, 45)
    '''

currentTime = time.time()
mission()
print(time.time()-currentTime)
