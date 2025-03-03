import time
import pigpio 
import numpy as np
import csv

import motors_ctr
import sonar_trigger_echo
import PIDController as PID

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
motors = motors_ctr.Motors(pi1, 
                            17, 27, 22, 10, # North Motors
                            9, 11, 5, 6,    # South Motors 
                            21, 20, 16, 12, 1, 7, 8, 25)         # Encoders

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

    # Assigning parameter values
    # Pulses Per Revolution of the encoder, 
    # calculated by counting the pulses each inerupt the hall effect sensors make 
    ppr = 543 
    tsample = 0.02  # Sampling period for code execution (s)

    t = []
    v = []
    # feedback loop testing params
    target_w = 2*np.pi # target value for angular velocity to be kept as reference
    target_pwm = 127  # starting value for PWM to be increaced/decreased

    # PID CONTROLLER SETUP
    setpoint = 2*np.pi  # Desired temperature
    pid = PID.PIDController(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=setpoint)
    process_variable = np.pi
    dt = 0.02
    # Execution main loop that displays the current
    # angular position of the encoder shaft, and other velocity info
    while motors.encoderNL.steps < 10000:

        # DUMB FEEDBACK LOOP A)
        # set target motors angular velocity to target
        #motors.change_dc(target_pwm) # change duty cycle 
        motors.W(w1=process_variable, w2=process_variable ,w3=process_variable, w4=process_variable)

        # PID FEEDBACK
        control_output = pid.compute(process_variable, dt)
        process_variable += control_output * dt - 0.1 * (process_variable - np.pi) * dt  # Heat loss

        # DUMB FEEDBACK LOOP B)
        speed_w = motors.get_speed(0) # angular_velocity = (ang2 - ang1)/tsample
        print(process_variable)

        # DUMB FEEDBACK LOOP C)
        # check actual measured speed and keep as close to it as we can
        if speed_w < target_w:
            target_pwm += 1
        else:
            target_pwm -= 1

        #print("steps = {}\nAngle = {:0.0f} deg".format(motors.encoderNL.steps, motors.get_degs(0)))
        print("NL: rads = {} rads/s Deg =  {}".format(motors.get_speed(0), motors.get_degs(i=0)))
        print("NR: rads = {} rads/s Deg =  {}".format(motors.get_speed(1), motors.get_degs(i=1)))
        print("SL: rads = {} rads/s Deg =  {}".format(motors.get_speed(2), motors.get_degs(i=2)))
        print("SR: rads = {} rads/s Deg =  {}".format(motors.get_speed(3), motors.get_degs(i=3)))

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