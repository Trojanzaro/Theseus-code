import pigpio
import time
import numpy as np

class Motors:
    """The control class for Theseus motors"""

    """Motor PWM duty dycle """
    pwm_dc = 127
    pos =0

    def __init__(self, pi1, NLL, NLH, NRL, NRH, SLL, SLH, SRL, SRH, NLA, NLB):
        """The pigpio object for controlling the GPIOS"""
        self.pi = pi1

        """North Left Motor"""
        self.NLL = NLL
        self.NLH = NLH

        """North Right Motor"""
        self.NRL = NRL
        self.NRH = NRH

        """South Left Motor"""
        self.SLL = SLL
        self.SLH = SLH

        """South Right Motor"""
        self.SRL = SRL
        self.SRH = SRH

        """MOTOR ENC PHASES"""

        """North Left Motor Phase"""
        self.NLA = NLA
        self.NLB = NLB
        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        # setup all pins
        # MOTOR Vs/GND/PWM
        self.pi.set_mode(NLL, pigpio.OUTPUT) # GPIO 17, 27, 22,  10, 9, 11,  5, 6  as output
        self.pi.set_mode(NLH, pigpio.OUTPUT)
        self.pi.set_mode(NRL, pigpio.OUTPUT)
        #
        self.pi.set_mode(NRH, pigpio.OUTPUT)
        self.pi.set_mode(SLL, pigpio.OUTPUT)
        self.pi.set_mode(SLH, pigpio.OUTPUT)
        #
        self.pi.set_mode(SRL, pigpio.OUTPUT)
        self.pi.set_mode(SRH, pigpio.OUTPUT)

        #MOTORS ENCODERS PHASES
        self.pi.set_mode(NLA, pigpio.INPUT)
        self.pi.set_mode(NLB, pigpio.INPUT)
        self.pi.set_pull_up_down(NLA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(NLB, pigpio.PUD_UP)
        self.cbA = self.pi.callback(NLA, pigpio.EITHER_EDGE, self._pulseA)
        self.cbB = self.pi.callback(NLB, pigpio.EITHER_EDGE, self._pulseA)

    def callback(self, way):
        self.pos += way
        print("pos={}".format(self.pos))

    def _pulseA(self, gpio, level, tick):
        #print(gpio, level, tick)
        if gpio == self.NLA:
            self.levA = level
        else:
            self.levB = level

        if gpio != self.lastGpio: # debounce
            self.lastGpio = gpio

            if gpio == self.NLA and level == 1:
                if self.levB == 1:
                    self.callback(1)
            elif gpio == self.NLB and level == 1:
                if self.levA == 1:
                    self.callback(-1)


    # change DutyCycle
    def change_dc(self, dc):
        # ! V = 7.7 100% DutyCycle -> ~21-22 cm/s
        # ! V = 7.7 50% DutyCycle -> ~12-13 cm/s
        # ! V = 7.7 25% DutyCycle -> ~5-6 cm/s
        self.pwm_dc = dc

    # return PWM for rad/s
    def rads2pwm(self, rads=None):
        #pwm_dc = (rads + 49.91) / 2.096 # ! Magic numbers come from the statistical analysis of the motors PWM to rad/s, linear regresion f(x) = 0.034x - 0.5
        return np.ceil(np.interp(rads, [0, (np.pi*5)/2], [0, 255])) if rads != None else self.pwm_dc


    def HALT(self):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.write(self.NLH, 0) # -
        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0) # -

        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.write(self.SLH, 0) # -
        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)  # -

    def NORTH(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads)) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(rads)) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(rads))  # SL
        self.pi.write(self.SLH, 0)

        self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(rads))  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def SOUTH(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0)               # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads)) # -

        self.pi.write(self.NRL, 0)               # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(rads)) # -    

        # South wheels
        self.pi.write(self.SLL, 0)                # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(rads)) # -

        self.pi.write(self.SRL, 0)                # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(rads))  # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CLOCK(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads)) # NL
        self.pi.write(self.NLH, 0)              # -

        self.pi.write(self.NRL, 0)                  # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(rads)) #-

        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(rads))       # SL
        self.pi.write(self.SLH, 0)                 # -

        self.pi.write(self.SRL, 0)                 # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(rads)) # -

        # delay for distance (* .5 sec for ~30 deg)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CLOCK_P(self,  dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads))              # -

        self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(rads))                  # NR
        self.pi.write(self.NRH, 0) #-

        # South wheels
        self.pi.write(self.SLL, 0)       # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(rads))                 # -

        self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(rads))                 # SR
        self.pi.write(self.SRH, 0) # -

        # delay for distance (* .5 sec for ~30 deg)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def STRAFE_RIGHT(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads))   # NL
        self.pi.write(self.NLH, 0)                    # -

        self.pi.write(self.NRL, 0)                    # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(rads))   # -
        
        # South wheels
        self.pi.write(self.SLL, 0)                     # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(rads))

        self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(rads))    # SR
        self.pi.write(self.SRH, 0)                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def STRAFE_LEFT(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0)   # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads))                    # -

        self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(rads))                    # NR
        self.pi.write(self.NRH, 0)   # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(rads))                     # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)    # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(rads))                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_RIGHT(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads)) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(rads))  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_RIGHT_P(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads))               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(rads))                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()
    
    def DIAGONALY_LEFT(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(rads)) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(rads))  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_LEFT_P(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(rads))               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(rads))

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def BEND_CLOCK(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads)) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(rads))  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def BEND_CLOCK_P(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0)               # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads)) # -

        self.pi.write(self.NRL, 0)               # NR
        self.pi.write(self.NRH, 0) # -    

        # South wheels
        self.pi.write(self.SLL, 0)                # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(rads)) # -

        self.pi.write(self.SRL, 0)                # SR
        self.pi.write(self.SRH, 0)  # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CENTER_CLOCK(self, dt=None, rads=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(rads))   # NL
        self.pi.write(self.NLH, 0)                    # -

        self.pi.write(self.NRL, 0)                    # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(rads))   # -
        
        # South wheels
        self.pi.write(self.SLL, 0)                     # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)    # SR
        self.pi.write(self.SRH, 0)                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CENTER_CLOCK_P(self, dt=None, rads=None):
        # North wheels
        self.pi.write(self.NLL, 0)   # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(rads))                    # -

        self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(rads))                    # NR
        self.pi.write(self.NRH, 0)   # -
        
        # South wheels
        self.pi.write(self.SLL, 0)                     # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)    # SR
        self.pi.write(self.SRH, 0)                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def W(self, dt=None, w1=None, w2=None, w3=None, w4=None):
        # North wheels
        if w1 > 0:
            self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(w1)) # NL
            self.pi.write(self.NLH, 0)               # -
        else:
            self.pi.write(self.NLL, 0)               # NL
            self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(w1*(-1))) # -

        if w2 > 0:
            self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(w2)) # NR
            self.pi.write(self.NRH, 0)               # -
        else:
            self.pi.write(self.NRL, 0)               # NR
            self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(w2*(-1))) # - 
        

        # South wheels
        if w3 > 0:
            self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(w3))  # SL
            self.pi.write(self.SLH, 0)
        else:
            self.pi.write(self.SLL, 0)                # SL
            self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(w3*(-1))) # -

        if w4 > 0:
            self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(w4))  # SR
            self.pi.write(self.SRH, 0)                # -
        else:
            self.pi.write(self.SRL, 0)                 # SR
            self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(w4*(-1))) # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()