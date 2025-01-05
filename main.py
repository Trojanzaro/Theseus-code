import time
import sys
import pigpio

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
        motors.CLOCK_P(1.60) # Δt ~= 1.45 sec =~ 90° 
        time.sleep(1) # wait for one second

        # array for the distances
        dist = []

        # START TAKING DISTANCE MEASURE MENTS AND APPEND THEM IN THE dist ARRAY
        n = 116 # 45 steps of 2° rotation for 90° rotation | 116 steps ~180° rotation
        while n > 0:

            # GET A READING AND ADD IT TO THE DISTANCE ARRAY
            r = sonar.read()
            print("D: {}".format(r))
            dist.append(r)

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
        theta = np.deg2rad(np.arange(180, 0, -1.5652173913)) # magic number necesary for 116 steps

        # plotting the polar coordinates on the system
        plt.polar(theta, dist, marker='o')

        # Setting the axis limit
        #ax.set_ylim(0,10)

        plt.savefig("map.png")

        print(dist)





except KeyboardInterrupt:
    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()
