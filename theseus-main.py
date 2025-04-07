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
dt = 0.01
setpoint = 2*np.pi  # Desired speed in rads/s

pid_nl = PID.PIDController(Kp=1.3, Ki=1.72, Kd=0.7, setpoint=setpoint)
pid_nr = PID.PIDController(Kp=5.5, Ki=9.82, Kd=0.55, setpoint=setpoint)
pid_sl = PID.PIDController(Kp=5.5, Ki=9.82, Kd=0.55, setpoint=setpoint)
pid_sr = PID.PIDController(Kp=5.5, Ki=9.82, Kd=0.55, setpoint=setpoint)

process_variable_nl = np.pi
process_variable_nr = np.pi
process_variable_sl = np.pi
process_variable_sr = np.pi

# function that reads the speeds from the serial port
def getSpeeds():
    if port.in_waiting > 0:
        rcv = port.readline().decode('ascii').rstrip().split(" ")
        angular_velocities = list(map(float, rcv))
        return angular_velocities

# function for setting all motors speeds at the same time
# NOTE: the SET_SPEED bollean is meant to have a 100ms delay before being set to False again
def setSpeeds(rads: list):
    if port.isOpen():
        port.write((" ".join(map(str, rads)) + '\n').encode())

# function for setting one motor speed at a time
# NOTE: the SET_SPEED bollean is meant to have a 100ms delay before being set to False again
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
        print('writting to file!')
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})

# MAIN LOOP STARTS WITH TRY CATCH
# MAIN LOOP
try:
    # start the speed thread to monitor and change the target speeds
    sd_thr = Thread(target=speed_thread, args=(t1, port), daemon=True)
    sd_thr.start()

    while True:
        print(w)
        # PID FEEDBACK LOOP A)
        # set target motors angular velocity to target
        setSpeed(process_variable_nl, 0)
        time.sleep(0.5)

        # PID FEEDBACK LOOP B)
        # compute error
        control_output_nl = pid_nl.compute(process_variable_nl, dt)
        process_variable_nl += control_output_nl * dt

        if time.time() - t1 >= 4 and time.time() - t1 < 8:
            pid_nl.set(3.14)
        
        if time.time() - t1 > 8:
            pid_nl.set(-3.14)

except:
    setSpeeds([0,0,0,0])
    print("error, f u")
    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})