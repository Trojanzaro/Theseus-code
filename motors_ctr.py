import pigpio
import time
import numpy as np
from gpiozero import RotaryEncoder

class Motors:
    """The control class for Theseus motors"""

    """Motor PWM duty dycle """
    pwm_dc = 127

    """Wheel position counter per pulse"""
    wheel_pulse_count = 0

    """ Motor encoder output pulses per 360 degree revolution (measured manually) ppr or Pulses Per Revolution of the encoder """
    ENC_PPR = 543 # calculated to be 543 for 360 degrees of rotation

    """Encoder object"""
    encoderNL = None
    encoderNR = None
    encoderSL = None
    encoderSR = None

    """Encoder steps"""
    sEncoderNL = 0
    sEncoderNR = 0
    sEncoderSL = 0
    sEncoderSR = 0

    # Variable for RPM measuerment
    rpm_right = 0.0
    
    # Variable for angular velocity measurement
    ang_velocity_NL = 0.0
    
    # CONSTANTS FOR CALCULATIONS
    rpm_to_radians = 0.10471975512
    rad_to_deg = 57.29578

    def __init__(self, pi1, NLL, NLH, NRL, NRH, SLL, SLH, SRL, SRH, NLA, NLB, NRA=None, NRB=None, SLA=None, SLB=None, SRA=None, SRB=None):
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
        #MOTOR ENCODERS 
        self.encoderNL = RotaryEncoder(NLA, NLB, max_steps=0)
        #self.encoderNR = RotaryEncoder(NRA, NRB, max_steps=0)
        #self.encoderSL = RotaryEncoder(SLA, SLB, max_steps=0)
        #self.encoderSR = RotaryEncoder(SRA, SRB, max_steps=0)

    # change DutyCycle
    def change_dc(self, dc):
        # ! V = 7.7 100% DutyCycle -> ~21-22 cm/s
        # ! V = 7.7 50% DutyCycle -> ~12-13 cm/s
        # ! V = 7.7 25% DutyCycle -> ~5-6 cm/s
        self.pwm_dc = dc

    # return PWM for rad/s
    def rads2pwm(self, rads=None):
        # try to keep the pwm close to the rad/s that we want
        # TODO: Implement
        pwm = 64
        return self.pwm_dc

    def get_rads(self, i=0, th=None):
        if th!= None:
            return th*0.017453
        if i == 0:
            return (360 / self.ENC_PPR * self.encoderNL.steps)*0.017453
        if i == 1:
            pass #return 360 / self.ENC_PPR * self.encoderNR.steps
        if i == 2:
            pass #return 360 / self.ENC_PPR * self.encoderSL.steps # return the angle the ith wheel is at by calculating (360 / encoders_pulses_per_revolution * encoder_steps)
        if i == 3:
            pass #return 360 / self.ENC_PPR * self.encoderSR.steps

    def get_degs(self, i=0, th=None):
        if th!= None:
            return th/0.017453
        return self.get_rads(i)/0.017453
    
    def get_speed(self, i=0):
        tsample = 0.02
        distance1 = self.get_rads(i)
        time.sleep(tsample)
        distance2 = self.get_rads(i)
        return (distance2-distance1)/tsample

    def W(self, dth=None, w1=None, w2=None, w3=None, w4=None):
        # North wheels
        if w1 > 0:
            self.pi.set_PWM_dutycycle(self.NLL, self.rads2pwm(w1)) # NL
            self.pi.write(self.NLH, 0)               # -
        elif w1 < 0:
            
            self.pi.write(self.NLL, 0)               # NL
            self.pi.set_PWM_dutycycle(self.NLH, self.rads2pwm(w1*(-1))) # -
        else:
            self.pi.write(self.NLH, 0) # -
            self.pi.write(self.NLL, 0)  

        if w2 > 0:
            self.pi.set_PWM_dutycycle(self.NRL, self.rads2pwm(w2)) # NR
            self.pi.write(self.NRH, 0)               # -
        elif w2 < 0:
            self.pi.write(self.NRL, 0)               # NR
            self.pi.set_PWM_dutycycle(self.NRH, self.rads2pwm(w2*(-1))) # - 
        else:
            self.pi.write(self.NRL, 0)
            self.pi.write(self.NRH, 0) # - 


        # South wheels
        if w3 > 0:
            self.pi.set_PWM_dutycycle(self.SLL, self.rads2pwm(w3))  # SL
            self.pi.write(self.SLH, 0)
        elif w3 < 0:
            self.pi.write(self.SLL, 0)                # SL
            self.pi.set_PWM_dutycycle(self.SLH, self.rads2pwm(w3*(-1))) # -
        else:
            self.pi.write(self.SLL, 0)                # SL
            self.pi.write(self.SLH, 0) # -

        if w4 > 0:
            self.pi.set_PWM_dutycycle(self.SRL, self.rads2pwm(w4))  # SR
            self.pi.write(self.SRH, 0)                # -
        elif w4 < 0:
            self.pi.write(self.SRL, 0)                 # SR
            self.pi.set_PWM_dutycycle(self.SRH, self.rads2pwm(w4*(-1))) # -
        else:
            self.pi.write(self.SRL, 0)                 # SR
            self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dth != None:
            time.sleep(dth)

            # halt motors
            self.HALT()


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

