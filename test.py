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
                            22, 10, 17, 27, # North Motors
                            5, 6, 9, 11,     # South Motors 
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
    st =[]

    # PID CONTROLLER SETUP
    setpoint = np.pi  # Desired speed in rads/s

    pid_nl = PID.PIDController(Kp=5.5, Ki=0.82, Kd=0.85, setpoint=setpoint)
    pid_nr = PID.PIDController(Kp=5.5, Ki=0.82, Kd=0.85, setpoint=setpoint)
    pid_sl = PID.PIDController(Kp=5.5, Ki=0.82, Kd=0.85, setpoint=setpoint)
    pid_sr = PID.PIDController(Kp=5.5, Ki=0.82, Kd=0.85, setpoint=setpoint)
    
    process_variable_nl = 2*np.pi
    process_variable_nr = 2*np.pi
    process_variable_sl = 2*np.pi
    process_variable_sr = 2*np.pi

    dt = 0.02
    # Execution main loop that displays the current
    # angular position of the encoder shaft, and other velocity info
    b = time.time()
    while motors.sEncoderNL < 5000:

        # PID FEEDBACK LOOP A)
        # set target motors angular velocity to target
        #motors.W(w1=process_variable_nl, w2=process_variable_nr, w3=process_variable_sl, w4=process_variable_sr)
        motors.change_dc(50)
        motors.NORTH()
        
        # PID FEEDBACK LOOP B)
        # compute error
        control_output_nl = pid_nl.compute(process_variable_nl, dt)
        control_output_nr = pid_nr.compute(process_variable_nr, dt)
        control_output_sl = pid_sl.compute(process_variable_sl, dt)
        control_output_sr = pid_sr.compute(process_variable_sr, dt)

        # add to cumulative process value 
        process_variable_nl += control_output_nl * dt
        process_variable_nr += control_output_nr * dt
        process_variable_sl += control_output_sl * dt
        process_variable_sr += control_output_sr * dt

        #print("steps = {}\nAngle = {:0.0f} deg".format(motors.encoderNL.steps, motors.get_degs(0)))
        print("NL: rads = {} rads/s Deg =  {}".format(motors.speedNL , motors.get_degs(i=0)))
        print("NL: {}".format(motors.sEncoderNL))
        # print("NR: rads = {} rads/s Deg =  {}".format(motors.get_speed(1), motors.get_degs(i=1)))
        # print("SL: rads = {} rads/s Deg =  {}".format(motors.get_speed(2), motors.get_degs(i=2)))
        # print("SR: rads = {} rads/s Deg =  {}".format(motors.get_speed(3), motors.get_degs(i=3)))

        t.append(time.time() - b)
        v.append(motors.speedNL)
        st.append(motors.sEncoderNL)

    with open('eggs.csv', 'w', newline='') as csvfile:
        for i in range(0, len(t)):
            spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v', 's'])
            spamwriter.writerow({'t': t[i], 'v': v[i], 's':st[i]})
    print('Done.')
    # Releasing GPIO pins

    print("STOPPING!")
    motors.HALT()
    pi1.stop()

except KeyboardInterrupt:
    with open('eggs.csv', 'w', newline='') as csvfile:
        spamwriter = csv.DictWriter(csvfile, fieldnames=['t', 'v'])
        spamwriter.writerow({'t': motors.sEncoderNL, 'v': motors.speedNL})

    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()