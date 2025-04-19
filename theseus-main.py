import serial, time 
import numpy as np
import csv

import pigpio
import sonar_trigger_echo

from threading import Thread

# serial communication port connected at pins GPIO14 GPIO15 (Tx,Rx)
# that are directly connected to the Arduino UNO's pins 0, 1 (Rx,Tx) 
port = serial.Serial(port="/dev/ttyS0", 
                     baudrate=115200, 
                     timeout=3)

# PIGPIO object
pi = pigpio.pi()

# create sonar HC-SR04 object
sonar = sonar_trigger_echo.ranger(pi, 23, 18)

# angular_speeds_thread angular velocites array
# will be set to the arduino read angular velocities of each motor 0,1,2,3
w = [0.0, 0.0, 0.0, 0.0]

# CONTROLLER SETUP
dt = 0.05
setpoint = [0.0, 0.0, 0.0, 0.0]  # Desired angular speed in rads/s
process_variables = [0.0, 0.0, 0.0, 0.0] # actual output angular speeds in rads/s

# speed array for [Vy, Vx, Ωz]
v = [0.0, 0.0, 0.0] 

# function for setting all motors angular velocities at the same time
def set_angular_ws(rads: list):
    if port.isOpen():
        port.write((" ".join(map("{:.2f}".format, rads)) + '\n').encode())

# function for setting one motor angular velocity at a time
def set_angular_w(rads: float, i: int):
    w = [-9999, -9999, -9999, -9999] # set to magic value -9999, if arduino reads this value as one of the speeds it will not change it
    w[i] = rads # change the motor we want to shange
    if port.isOpen():
        port.write((" ".join(map("{:.2f}".format, w)) + '\n').encode())

# resets the global move_true and collide_true variables after set timeout
def rese_timeout(sec):
    global move_true, collide_true
    dt1=time.time()
    dt2=time.time()
    while dt2 - dt1 <= sec:
        dt2=time.time()
    move_true = True
    collide_true = False

# angular_speeds_thread thread serves two purposes, to read the speeds from the arduino through
# serial and setting the w variable to the responding speeds 
def angular_speeds_thread(port):
    global w
    while True:
        if port.in_waiting > 0:
            rcv = port.readline().decode().rstrip().split(" ")
            angular_velocities = list(map(float, rcv))
            w = angular_velocities


# MAIN LOOP STARTS WITH TRY CATCH
# MAIN LOOP
try:
    # start the speed thread to monitor and change the target speeds
    sd_thr = Thread(target=angular_speeds_thread, args=(port,), daemon=True)
    sd_thr.start()

    # set inverse kinematics input array [Vx, Vy, Wz]
    r = 0.033 # radius of on wheel 36 mm
    lx = 0.65
    ly = 0.7 # 5 cm + 35/2 mm

    # move bool, if true the robot moves forward only [Vy=0.1 cm/s, Vx=0, Ωz=0]
    move_true = True

    # collider bool, if true it means sonar passed the threashhold distance ~10cm
    collide_true = False
    
    # velocity inverse kinematics [Vy, Vx, Ωz]
    v = [0.1, 0.0, 0.0]

    # LOOP
    while True:
        
        # if the move_true boolean is true, keep the robot walking forward with 10cm/s speed
        if move_true:
            v = [0.1, 0.0, 0.0]
        
        # continiously monitor rover's distance
        # if rover distance passes thresshold then change speeds to avoid obstancle
        if (sonar.read() / 2) / 29.1 <= 11.001 and not collide_true:

            # booleans change for only changing the speeds once
            collide_true = True
            move_true = False
            
            # turn Counter Clockwise 0.11 rads/s
            v= [0.0, 0.0, 0.1]

            # start the reset_timeout thread and have it wait for 1.0 seconds  before
            # it resets the move_true and collide_true booleans
            move_thread = Thread(target=rese_timeout, args=(1, ), daemon=True)
            move_thread.start()

        # INVERSE KINEMATICS
        # Construct the transformation matrix
        transform_matrix = (1 / r) * np.array([
            [ 1, -1, -(lx + ly)],
            [ 1,  1,  (lx + ly)],
            [ 1,  1, -(lx + ly)],
            [ 1, -1,  (lx + ly)]
        ])

        # Compute the angular velocities
        omega = np.dot(transform_matrix, np.array(v))

        # set angukar velocities
        setpoint = omega

        # FEEDBACK LOOP A)
        # set target motors angular velocity to target
        set_angular_ws(process_variables)
        time.sleep(dt)

        # FEEDBACK LOOP B)
        # compute error
        control_output_nl = setpoint[0] - w[0]
        control_output_nr = setpoint[1] - w[1]
        control_output_sl = setpoint[2] - w[2]
        control_output_sr = setpoint[3] - w[3]

        # FEEDBACK LOOP C)
        # FEEDBACK!
        process_variables[0] += control_output_nl *dt*5.2
        process_variables[1] += control_output_nr *dt*5.2
        process_variables[2] += control_output_sl *dt*5.2
        process_variables[3] += control_output_sr *dt*5.2
except:
    sonar.cancel()
    pi.stop()
    set_angular_ws([0,0,0,0])
    print("error, f u")