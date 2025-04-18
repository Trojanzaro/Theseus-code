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

# this is a time counter to measure runtime elapsed time from
# program start
t1 = time.time()

# speed thread speeds array
w = []

# CONTROLLER SETUP
dt = 0.05
setpoint = [6.28, 6.28, 6.28, 6.28]  # Desired angular speed in rads/s

process_variables = [0, 0, 0, 0] # actual output angular speeds in rads/s

v = [0.0, 0.0, 0.0] # speed array for [Vy, Vx, Ωz]

# function for setting all motors speeds at the same time
def setSpeeds(rads: list):
    if port.isOpen():
        port.write((" ".join(map("{:.2f}".format, rads)) + '\n').encode())

# function for setting one motor speed at a time
def setSpeed(rads: float, i: int):
    w = [-9999, -9999, -9999, -9999]
    w[i] = rads
    if port.isOpen():
        port.write((" ".join(map("{:.2f}".format, w)) + '\n').encode())

# function for moving the robot
def move(v_array=[], dt=None):
    global v
    v = v_array
    if dt is not None:
        time.sleep(dt)
        v = [0.0, 0.0, 0.0]

def get_speed(sonar, move_thread=None):
    global v
    while True:
        d1 = (sonar.read()/2)/29.1
        time.sleep(0.5)
        d2 = (sonar.read()/2)/29.1

        print("{} {}".format(d2, (d1-d2)/0.5))

# the speed thread serves two purposes, to read the speeds from the arduino through
# serial 
def speed_thread(lt1, port):
    global w
    while True:
        if port.in_waiting > 0:
            rcv = port.readline().decode('ascii').rstrip().split(" ")
            angular_velocities = list(map(float, rcv))
            w = angular_velocities


# MAIN LOOP STARTS WITH TRY CATCH
# MAIN LOOP
try:
    # PIGPIO
    pi = pigpio.pi()

    # create sonar object
    sonar = sonar_trigger_echo.ranger(pi, 23, 18)

    # start the speed thread to monitor and change the target speeds
    sd_thr = Thread(target=speed_thread, args=(t1, port), daemon=True)
    sd_thr.start()

    # set inverse kinematics input array [Vx, Vy, Wz]
    r = 0.033 # radius of on wheel 36 mm
    lx = 0.65
    ly = 0.7 # 5 cm + 35/2 mm

    # move bool
    move_b = True
    move_thread = Thread(target=move, args=([0.1, 0.0, 0.0], ), daemon=True)
    move_thread.start()

    while True:
        
        # DEMO MOVE THREAD
        # MOVE THREAD

        print("{} {}".format(r, (sonar.read()/2)/29.1))
        if (sonar.read()/2)/29.1 <= 11.001:
            print("collide")
            # move_thread = Thread(target=move, args=(1, [0.0, 0.0, 0.16]), daemon=False)
            # move_thread.start()

        
        # DEMO - INVERSE KINEMATICS
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
        setSpeeds(process_variables)
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
    setSpeeds([0,0,0,0])
    print("error, f u")