#Go to pwr plant
    abs_flip_turn(flipper, 55, 30, flipperInit)
    highspeed_pid(hub, robot, 50.5, 80, 0)
    abs_flip_turn(flipper, 0, 60, flipperInit)

    #Turn right 35, and go diagonal
    abs_turning(hub, robot, 60, 10)
    pid(hub, robot, 6, 50, 60)

    #Turn back to 0 b4 pushing thing down then push down
    abs_turning(hub, robot, 0, 30)

    #Go more B4 pushing down
    pid(hub, robot, 8, 10, 0)
    abs_flip_turn(flipper, 90, 30, flipperInit)

    #Go HOME
    highspeed_pid(hub, robot, 76, -80, 0)

    #Turn for TV
    abs_turning(hub, robot, 90, 30)

    #Tv mission go forward and come back
    pid(hub, robot, 33, 50, 90)
    highspeed_pid(hub, robot, 23.75, -80, 90)
    abs_flip_turn(flipper, 0, 30, flipperInit)
    return
