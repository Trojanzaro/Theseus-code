import time
import pigpio 
import numpy as np
import csv

import motors_ctr
import sonar_trigger_echo

# setup PIGPIO
pi1 = pigpio.pi()       # pi1 accesses the local Pi's GPIO

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

# Initialize the motors GPIOs 
motors = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6, 12, 16)

# setup HC-SR04 ultranosic sensor
sonar = sonar_trigger_echo.ranger(pi1, 23, 18)

####
# MAIN LOOP
####
try:
    # np.array([
    #         [1, -1, -(lx + ly)], 
    #         [1,  1,  (lx + ly)], 
    #         [1,  1, -(lx + ly)], 
    #         [1, -1,  (lx + ly)]
    #     ]
    # )

    T = np.array([
            [1, -1, -(6.1 + 6.2)], 
            [1,  1, (6.1 + 6.2)], 
            [1,  1, -(6.1 + 6.2)], 
            [1, -1, (6.1 + 6.2)]
        ]
    )
    T_P = np.array([
            [1, 1, 1, 1], 
            [-1,  1, 1, -1], 
            [-1/(6.1 + 6.2),  1/(6.1 + 6.2), -1/(6.1 + 6.2), 1/(6.1 + 6.2)]
        ]
    )
    v = np.array([-12.7, 10, 0]) # testing forward kinematics: Vx = 1.1, Vy = 0, Ωz = 0 
    
    #T = np.multiply(1/0.33, T)
    #T = np.multiply(T, v)

    w1=(1/3.3)*(v[0]-v[1]-(6.1 + 6.2)*v[2]) # forward kinematics test
    w2=(1/3.3)*(v[0]+v[1]+(6.1 + 6.2)*v[2])
    w3=(1/3.3)*(v[0]+v[1]-(6.1 + 6.2)*v[2])
    w4=(1/3.3)*(v[0]-v[1]+(6.1 + 6.2)*v[2])

    #T_P = np.multiply(0.33/4, T_P)
    print(T, w1, w2, w3, w4)
    motors.change_dc(64)
    motors.HALT()

    # Assigning parameter values
    # Pulses Per Revolution of the encoder, 
    # calculated by counting the pulses each inerupt the hall effect sensors make 
    ppr = 543  
    tstop = 10  # Loop execution duration (s)
    tsample = 0.02  # Sampling period for code execution (s)
    tdisp = 0.5  # Sampling period for values display (s)

    anglecurr = 0
    tprev = 0
    tcurr = 0
    tstart = time.perf_counter()

    # Execution loop that displays the current
    # angular position of the encoder shaft
    print('Running code for', tstop, 'seconds ...')
    print('(Turn the encoder.)')
    while True:
        # Pausing for `tsample` to give CPU time to process encoder signal
        time.sleep(tsample)
        # Getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Printing angular position every `tdisp` seconds
        if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
            print("Angle = {:0.0f} deg".format(motors.get_angle(0)))
        # Updating previous values
        tprev = tcurr

    print('Done.')
    # Releasing GPIO pins

    # Assigning parameter values
    T = 2  # Period of sine wave (s)
    u0 = 1  # Motor output amplitude
    tstop = 4  # Sine wave duration (s)
    tsample = 0.01  # Sampling period for code execution (s)
    # Pre-allocating output arrays
    t = []
    theta = []

    # Initializing current time step and starting clock
    tprev = 0
    tcurr = 0
    tstart = time.perf_counter()

    # Running motor sine wave output
    print('Running code for', tstop, 'seconds ...')
    while tcurr <= tstop:
        # Pausing for `tsample` to give CPU time to process encoder signal
        time.sleep(tsample)
        # Getting current time (s)
        tcurr = time.perf_counter() - tstart
        # Assigning motor sinusoidal output using the current time step
        motors.W(w1=0, w2=0, w3=u0 * np.sin((2*np.pi/T) * tcurr), w4=0)
        # Updating output arrays
        t.append(tcurr)
        #theta.append(mymotor.get_angle())
        # Updating previous time value
        tprev = tcurr
        with open('eggs.csv', 'a', newline='') as csvfile:
            spamwriter = csv.DictWriter(csvfile, fieldnames=['sin(x)', 'pos'])
            spamwriter.writerow({'sin(x)': u0 * np.sin(2*np.pi/T) * tcurr, 'pos': motors.pos})

    print('Done.')
    # Stopping motor and releasing GPIO pins
    motors.HALT()

    # Calculating motor angular velocity (rpm)
    #w = 60/360 * np.gradient(theta, t)


    # Plotting results
    #plot_line([t, t, t[1::]], [theta, w, 1000*np.diff(t)], axes='multi',yname=['Angular Position (deg.)','Angular velocity (rpm)','Sampling Period (ms)'])

    encoder.close()

    print("STOPPING!")
    motors.HALT()
    pi1.stop()

except KeyboardInterrupt:
    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()
