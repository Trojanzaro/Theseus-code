import serial, time 
import numpy as np
import PIDController as PID # TODO: Discuss removal
import csv, io
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
setpoint = [6.28, 6.28, 6.28, 6.28]  # Desired speed in rads/s

process_variables =[0, 0, 0, 0] # actual output speeds in rads/s

# function that reads the speeds from the serial port
def getSpeeds():
    if port.in_waiting > 0:
        rcv = port.readline().decode('ascii').rstrip().split(" ")
        angular_velocities = list(map(float, rcv))
        return angular_velocities

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

# the speed thread serves two purposes, to read the speeds from the arduino through
# serial 
def speed_thread(lt1, port):
    global w
    t = []
    v1 = []
    v2 = []
    v3 = []
    v4 = []

    while time.time() - lt1 <= 26:
        if port.in_waiting > 0:
            t.append(time.time() - lt1)
            rcv = port.readline().decode('ascii').rstrip().split(" ")
            angular_velocities = list(map(float, rcv))
            w = angular_velocities
            v1.append(angular_velocities[0])
            v2.append(angular_velocities[1])
            v3.append(angular_velocities[2])
            v4.append(angular_velocities[3])
    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})
        print('filewritten!')

# this is the Naive Error Calculator controller that only
# returns an error times a multiplier (past a threshold 0.9 error)
# so a single Porpotional controller (No I or D)
# TODO: DISCUSS!
def nv_error(ppoint, stpoint):
    error = stpoint - ppoint
    if error >= 0.9:
        return error*21
    else:
        return error*21

# MAIN LOOP STARTS WITH TRY CATCH
# MAIN LOOP
try:
    # start the speed thread to monitor and change the target speeds
    sd_thr = Thread(target=speed_thread, args=(t1, port), daemon=True)
    sd_thr.start()

    while True:
        print(w)
        # DEMO STEPS
        # STEPS
        if time.time() - t1 >= 3.9 and time.time() - t1 <= 4.0:
            process_variables = [0, 0, 0, 0]
            setpoint = [-6.28, -6.28, -6.28, -6.28]
        if time.time() - t1 >= 7.9 and time.time() - t1 <= 8.0:
            process_variables = [0, 0, 0, 0]
            setpoint = [6.28, -6.28, -6.28, 6.28]
        if time.time() - t1 >= 11.9 and time.time() - t1 <= 12:
            process_variables = [0, 0, 0, 0]
            setpoint = [-6.28, 6.28, 6.28, -6.28]
        if time.time() - t1 >= 16.9 and time.time() - t1 <= 17:
            process_variables = [0, 0, 0, 0]
            setpoint = [0, -6.28, -6.28, 0]

        # FEEDBACK LOOP A)
        # set target motors angular velocity to target
        setSpeeds(process_variables)
        time.sleep(dt)

        # FEEDBACK LOOP B)
        # compute error
        control_output_nl = nv_error(process_variables[0], setpoint[0])
        control_output_nr = nv_error(process_variables[1], setpoint[1])
        control_output_sl = nv_error(process_variables[2], setpoint[2])
        control_output_sr = nv_error(process_variables[3], setpoint[3])

        # FEEDBACK LOOP C)
        # FEEDBACK!
        process_variables[0] += control_output_nl * dt + w[0]*0.01
        process_variables[1] += control_output_nr * dt + w[1]*0.01
        process_variables[2] += control_output_sl * dt + w[2]*0.01
        process_variables[3] += control_output_sr * dt + w[3]*0.01


except:
    setSpeeds([0,0,0,0])
    print("error, f u")
    # with open('eggs.csv', 'w', newline='') as csvfile:
    #     for i in range(0, len(t)):
    #         spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
    #         spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})