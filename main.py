import time
import sys
import pigpio
import math  

import sonar_trigger_echo
import motors_ctr
import stage

# from PIL import Image, ImageDraw
# import PIL 

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

# setup PIGPIO
pi1 = pigpio.pi()       # pi1 accesses the local Pi's GPIO

####
# UTILITY FUNCTIONS
####

####
# sonar detection turn sequence
def turn_sequence(sonar):
    #  STOP
    motors.HALT()
    time.sleep(1)

    # TURN LEFT
    motors.CLOCK(0.5)
    
    # READ LEFT DISTANCE
    ld = sonar.read()
    
    # TURN RIGHT
    motors.CLOCK_P(1)

    # READ RIGHT DISTANCE
    rd = sonar.read()

    # TURN BACK AROUND
    motors.CLOCK(0.5)
    print("LD: {}, RD: {}".format(ld, rd))
    time.sleep(1)

    # if LD > RD TURN LEFT ELSE TURN RIGHT
    if ld >= rd:
        motors.CLOCK(1.5)
        time.sleep(0.2)
    else:
        motors.CLOCK_P(1.5)
        time.sleep(0.2)

####
# dithering startup
def dithering_startup(motors):
        motors.NORTH(0.1)
        time.sleep(0.2)
        motors.SOUTH(0.1)
        time.sleep(0.2)

        motors.SOUTH(0.1)
        time.sleep(0.2)
        motors.NORTH(0.1)
        time.sleep(0.2)

        motors.STRAFE_LEFT(0.1)
        time.sleep(0.2)
        motors.STRAFE_RIGHT(0.1)
        time.sleep(0.2)
        
        motors.STRAFE_RIGHT(0.1)
        time.sleep(0.2)
        motors.STRAFE_LEFT(0.1)

        time.sleep(2)

####
#callback function for terminating code
def cb_stop_bt(gpio, level, tick):
    print(gpio, level, tick)
    print("STOPPING APPLICATION")
    motors.HALT()
    pi1.stop()
    exit()

# interuopt GPIO for stopping by callback cb_stop_bt 
cb1 = pi1.callback(4, pigpio.EITHER_EDGE, cb_stop_bt)

# setup HC-SR04 ultranosic sensor
sonar = sonar_trigger_echo.ranger(pi1, 23, 18)

# Initialize the motors GPIOs 
motors = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6)

# Initialize the Stage the robot will move in
stg = stage.Stage(motors)

stg.print_stage()

####
# MAIN LOOP
####
try:
    while True:
    
        # read distance from ultrasonic sensor 
    
        #r = sonar.read()
        #print("D: {}".format(r))

        # if the distance is bellow threshold turn_sequence()
        #if r <= 1000:
        #    turn_sequence(sonar)
    
        # normal mode: walk forward
        #motors.NORTH()

        #stg.kernel_ctr()
        #time.sleep(0.5) # ~5cm per step
        
        ####
        # STARTING THE ROVER WITH A SMALL INTRI SEQUENSE TO START UP
        dithering_startup(motors)

        # START MEASURING DISTANCES
        # START WITH A 90° rotation
        motors.CLOCK_P(1.30) # Δt ~= 1.45 sec =~ 90° | ! 1.30 Δt with 9.0 V battery and 170 Duty Cycle
        time.sleep(1) # wait for one second

        # array for the distances
        dist = []

        # START TAKING DISTANCE MEASURE MENTS AND APPEND THEM IN THE dist ARRAY
        n = 73 # 45 steps of 2° rotation for 90° rotation | 116 steps ~180° rotation | ! 73 STEPS FOR 9.0 V battery and 170 Duty Cycle
        while n > 0:

            # GET A READING AND ADD IT TO THE DISTANCE ARRAY
            r = sonar.read() # read duration
            r = (r/2) / 29.1 # normalize duration to cm
            print("D: {}".format(r))
            dist.append(r) # add to distances array

            # TURN ROVER IN STEPS
            motors.CLOCK(0.05) # Δt = 5ms =~ 2°
            time.sleep(0.2)

            # START OVER
            n -= 1
        
        # CREATE A RADIO PLOT
        fig = plt.figure(dpi=200)
        ax = fig.add_subplot(projection='polar')
        plt.grid(True)

        # Generating the X and Y axis data point
        # Generating the θ range 180° - 0° 
        theta = np.deg2rad(np.arange(180, 0, -2.46575342466)) # magic number necesary for 116 steps | ! 73 steps

        # plotting the polar coordinates on the system
        plt.polar(theta, dist, marker='o')
        plt.savefig("map.png")

        # transformation from polar to cartesian coordinates
        fig = plt.figure(dpi=200)
        ax = fig.add_subplot()
        plt.grid(True)

        # where the coordinates will be stored
        x1 = []
        y1 = []

        # for the entire range transform from [r, θ] to [x, y]
        for i in range(0, 73):
            x = dist[i]*math.cos(theta[i]) # x = r1 cos(θ1)
            x1.append(x)

            y = dist[i]*math.sin(theta[i]) # y = r1 sin(θ1) 
            y1.append(y)

        # save plot 
        plt.plot(x1, y1, marker='o')
        plt.savefig("map2.png")


except KeyboardInterrupt:
    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()
