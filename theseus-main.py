import serial, time 
import numpy as np
import PIDController as PID
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

# PID CONTROLLER SETUP
dt = 0.05
setpoint = 6.28  # Desired speed in rads/s

pid_nl = PID.PIDController(Kp=1.095, Ki=1.778, Kd=0.72, setpoint=setpoint)
pid_nr = PID.PIDController(Kp=1.095, Ki=1.778, Kd=0.72, setpoint=setpoint)
pid_sl = PID.PIDController(Kp=1.095, Ki=1.778, Kd=0.72, setpoint=setpoint)
pid_sr = PID.PIDController(Kp=1.095, Ki=1.778, Kd=0.72, setpoint=setpoint)

process_variable_nl = 3.14 # starting value in rads/s
process_variable_nr = 3.14
process_variable_sl = 3.14
process_variable_sr = 3.14

# function that reads the speeds from the serial port
def getSpeeds():
    if port.in_waiting > 0:
        rcv = port.readline().decode('ascii').rstrip().split(" ")
        angular_velocities = list(map(float, rcv))
        return angular_velocities

# function for setting all motors speeds at the same time
def setSpeeds(rads: list):
    if port.isOpen():
        port.write((" ".join(map(str, rads)) + '\n').encode())

# function for setting one motor speed at a time
def setSpeed(rads: float, i: int):
    w = [-9999, -9999, -9999, -9999]
    w[i] = rads
    if port.isOpen():
        port.write((" ".join(map(str, w)) + '\n').encode())

# the speed thread serves two purposes, to read the speeds from the arduino through
# serial 
def speed_thread(lt1, port):
    global w
    t = []
    v1 = []
    v2 = []
    v3 = []
    v4 = []

    while time.time() - lt1 <= 20:
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
        if time.time() - t1 <= 4:
            pid_nl.set(6.28)
            pid_nr.set(6.28)
            pid_sl.set(6.28)
            pid_sr.set(6.28)    

        if time.time() - t1 >= 4 and time.time() - t1 <= 8:
            pid_nl.set(3.14)
            pid_nr.set(3.14)
            pid_sl.set(3.14)
            pid_sr.set(3.14)
        
        if time.time() - t1 >= 8 :
            pid_nl.set(6.28)
            pid_nr.set(-6.28)
            pid_sl.set(-6.28)
            pid_sr.set(6.28)

        # PID FEEDBACK LOOP A)
        # set target motors angular velocity to target
        setSpeeds([process_variable_nl, process_variable_nr, process_variable_sl, process_variable_sr])
        time.sleep(dt)

        # PID FEEDBACK LOOP B)
        # compute error
        control_output_nl = pid_nl.compute(process_variable_nl, dt)
        control_output_nr = pid_nr.compute(process_variable_nr, dt)
        control_output_sl = pid_sl.compute(process_variable_sl, dt)
        control_output_sr = pid_sr.compute(process_variable_sr, dt)

        # PID FEEDBACK LOOP C)
        # FEEDBACK!
        process_variable_nl += control_output_nl * dt
        process_variable_nr += control_output_nr * dt
        process_variable_sl += control_output_sl * dt
        process_variable_sr += control_output_sr * dt


except:
    setSpeeds([0,0,0,0])
    print("error, f u")
    # with open('eggs.csv', 'w', newline='') as csvfile:
    #     for i in range(0, len(t)):
    #         spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
    #         spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})