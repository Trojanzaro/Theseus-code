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
    tsample = 0.02  # Sampling period for code execution (s)

    # feedback loop testing params
    target_w = 2*np.pi # target value for angular velocity to be kept as reference
    target_pwm = 127  # starting value for PWM to be increaced/decreased

    t = []
    v = []

    # Execution loop that displays the current
    # angular position of the encoder shaft
    while motors.encoderNL.steps < 11879:

        # DUMB FEEDBACK LOOP A)
        # set target motors angular velocity to target
        motors.change_dc(target_pwm)
        motors.W(w1=target_w, w2=0 ,w3=target_w, w4=0)

        # Pausing for `tsample` to give CPU time to process encoder signal
        # ang1 = motors.get_rads(0)
        # time.sleep(tsample)
        # ang2 = motors.get_rads(0)

        # # DUMB FEEDBACK LOOP B)
        # # angular velocity calculation from actual values of motor encoders
        # ang_v = (ang2-ang1)/tsample

        # Printing angular information
        speed_w = motors.get_speed(0)
        print("steps = {}\nAngle = {:0.0f} deg".format(motors.encoderNL.steps, motors.get_degs(0)))
        print("rads = {} rads/s Deg =  {}".format(speed_w, motors.get_degs(th=speed_w)))

        # DUMB FEEDBACK LOOP C)
        # check actual measured speed and keep as close to it
        if speed_w < target_w:
            target_pwm += 1
        else:
            target_pwm -= 1
        t.append(motors.encoderNL.steps)
        v.append(motors.get_speed(0))

    with open('eggs.csv', 'a', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v'])
            spamwriter.writerow({'t': t[i], 'v': v[i]})
    print('Done.')
    # Releasing GPIO pins

    print("STOPPING!")
    motors.HALT()
    pi1.stop()

except KeyboardInterrupt:
    with open('eggs.csv', 'a', newline='') as csvfile:
        spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v'])
        spamwriter.writerow({'t': motors.encoderNL.steps, 'v': motors.get_speed(0)})

    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()