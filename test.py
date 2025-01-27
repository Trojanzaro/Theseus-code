import time
import pigpio 
import numpy as np

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
motors = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6)

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
    v = [1.1, 0, 0] # testing forward kinematics: Vx = 1.1, Vy = 0, Ωz = 0 
    T = np.multiply(1/0.33, T)
    T = np.multiply(T, v)

    w1=(1/0.33)*(v[0]-v[1]-(6.1 + 6.2)*T[0][1]) # forward kinematics test
    w2=(1/0.33)*(v[0]+v[1]+(6.1 + 6.2)*T[1][1])
    w3=(1/0.33)*(v[0]+v[1]-(6.1 + 6.2)*T[2][1])
    w4=(1/0.33)*(v[0]-v[1]+(6.1 + 6.2)*T[3][1])

    #T_P = np.multiply(0.33/4, T_P)
    print(T, w1,w2,w3,w4)
    while True:
        motors.W(w1=w1, w2=w2, w3=w3, w4=w4) # ! NEW FUNCTION FOR ALL MOTORs ANGULAR VELOCITY MOVE
        
        distance1 = (sonar.read()/2) / 29.1
        time.sleep(0.5)
        distance2 = (sonar.read()/2) / 29.1

        speed = (distance1 - distance2) / 0.5
        print("Speed: {}".format(speed))
        

    print("STOPPING!")
    motors.HALT()
    pi1.stop()

except KeyboardInterrupt:
    # close pigpio
    motors.HALT()
    print("STOPPING!")
    pi1.stop()
