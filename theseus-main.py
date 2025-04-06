import serial, time
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

# array for all Angular Velocities of each motor in rads/s
S_W = [0.0, 0.0, 0.0, 0.0]


P_W = [0.0, 0.0, 0.0, 0.0]

# boolean that is set every time we run the setSpeed() or setSpeeds() functions in
# order to halt altering the speeds before they have the time to get to the given speed before
# any corrections from the speed_thread()
# NOTE: starts as True to avoid sporadic motor speed changes at program start
SET_SPEED = True

# function for setting all motors speeds at the same time
# NOTE: the SET_SPEED bollean is meant to have a 100ms delay before being set to False again
def setSpeeds(rads: list):
    SET_SPEED = True
    P_W = rads
    S_W = rads
    if port.isOpen():
        port.write((" ".join(map(str, rads)) + '\n').encode())
    time.sleep(0.5)
    SET_SPEED = False

# function for setting one motor speed at a time
# NOTE: the SET_SPEED bollean is meant to have a 100ms delay before being set to False again
def setSpeed(rads: float, i: int):
    SET_SPEED = True
    P_W[i] = rads
    S_W[i] = rads
    w = [-9999, -9999, -9999, -9999]
    if i is None:
        if rads is not None:
            w = rads
        else:
            raise SyntaxError("one arghument is required")
    w[i] = rads
    if port.isOpen():
        port.write((" ".join(map(str, w)) + '\n').encode())
    time.sleep(0.5)
    SET_SPEED = False

# the speed thread serves two purposes, to read the speeds from the arduino through
# serial 
def speed_thread(lt1, port):
    t = []
    v1 = []
    v2 = []
    v3 = []
    v4 = []

    while time.time() - lt1 <= 26:
        if port.in_waiting > 0:
            t.append(time.time() - t1)
            rcv = port.readline().decode('ascii').rstrip().split(" ")
            angular_velocities = list(map(float, rcv))
            print(angular_velocities)

            # for each angular velocity received, adjust to reach it's requiested speed
            for i in range(0, 3):
                
                # if the read speed is lower thatn the set speed S_W then increase the process speed P_W and set the speed
                if angular_velocities[i] < S_W[i]: # if SET_SPEED is set to True that means we are still waitting for the setPseed/s() function to finish setting speeds
                    P_W[i] += 1.5
                    if not SET_SPEED:
                        setSpeed(P_W[i], i)
                
                # if the read speed is higher thatn the set speed S_W then decrease the process speed P_W and set the speed
                elif angular_velocities[i] > S_W[i]:
                    P_W[i] -= 1.5
                    if not SET_SPEED:
                        setSpeed(P_W[i], i)
            
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

    # DEMO
    setSpeeds([0, 0, 0, 0])
    time.sleep(4)
    setSpeed(6.28, 0)
    time.sleep(4)
    setSpeed(6.28, 1)
    time.sleep(4)
    setSpeed(6.28, 2)
    time.sleep(4)
    setSpeed(6.28, 3)
    time.sleep(4)
    setSpeeds([6.28, -6.28, -6.28, 6.28])
    time.sleep(4)
    setSpeeds([0, 0, 0, 0])
    time.sleep(4)

except Exception as ex:
    print("error, f u" + str(ex))
    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v1', 'v2', 'v3', 'v4'])
            spamwriter.writerow({'t': t[i], 'v1': v1[i],'v2': v2[i], 'v3': v3[i], 'v4': v4[i]})