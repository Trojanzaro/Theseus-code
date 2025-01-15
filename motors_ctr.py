import pigpio
import time

class Motors:
    """The control class for Theseus motors"""

    """Motor PWM duty dycle """
    pwm_dc = 255

    def __init__(self, pi1, NLL, NLH, NRL, NRH, SLL, SLH, SRL, SRH):
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

        # setup all pins
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

    def NORTH(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.set_PWM_dutycycle(self.NRL, self.pwm_dc) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.pwm_dc)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.set_PWM_dutycycle(self.SRL, self.pwm_dc)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def SOUTH(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0)               # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc) # -

        self.pi.write(self.NRL, 0)               # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.pwm_dc) # -    

        # South wheels
        self.pi.write(self.SLL, 0)                # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.pwm_dc) # -

        self.pi.write(self.SRL, 0)                # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.pwm_dc)  # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CLOCK(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc) # NL
        self.pi.write(self.NLH, 0)              # -

        self.pi.write(self.NRL, 0)                  # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.pwm_dc) #-

        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.pwm_dc)       # SL
        self.pi.write(self.SLH, 0)                 # -

        self.pi.write(self.SRL, 0)                 # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.pwm_dc) # -

        # delay for distance (* .5 sec for ~30 deg)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CLOCK_P(self,  dt=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc)              # -

        self.pi.set_PWM_dutycycle(self.NRL, self.pwm_dc)                  # NR
        self.pi.write(self.NRH, 0) #-

        # South wheels
        self.pi.write(self.SLL, 0)       # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.pwm_dc)                 # -

        self.pi.set_PWM_dutycycle(self.SRL, self.pwm_dc)                 # SR
        self.pi.write(self.SRH, 0) # -

        # delay for distance (* .5 sec for ~30 deg)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def STRAFE_RIGHT(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc)   # NL
        self.pi.write(self.NLH, 0)                    # -

        self.pi.write(self.NRL, 0)                    # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.pwm_dc)   # -
        
        # South wheels
        self.pi.write(self.SLL, 0)                     # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.pwm_dc)

        self.pi.set_PWM_dutycycle(self.SRL, self.pwm_dc)    # SR
        self.pi.write(self.SRH, 0)                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def STRAFE_LEFT(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0)   # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc)                    # -

        self.pi.set_PWM_dutycycle(self.NRL, self.pwm_dc)                    # NR
        self.pi.write(self.NRH, 0)   # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.pwm_dc)                     # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)    # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.pwm_dc)                     # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_RIGHT(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.set_PWM_dutycycle(self.SRL, self.pwm_dc)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_RIGHT_P(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.set_PWM_dutycycle(self.SRH, self.pwm_dc)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()
    
    def DIAGONALY_LEFT(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.set_PWM_dutycycle(self.NRL, self.pwm_dc) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.pwm_dc)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def DIAGONALY_LEFT_P(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.pwm_dc)               # -
        
        # South wheels
        self.pi.write(self.SLL, 0)  # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.pwm_dc)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def BEND_CLOCK(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc) # NL
        self.pi.write(self.NLH, 0)               # -

        self.pi.write(self.NRL, 0) # NR
        self.pi.write(self.NRH, 0)               # -
        
        # South wheels
        self.pi.set_PWM_dutycycle(self.SLL, self.pwm_dc)  # SL
        self.pi.write(self.SLH, 0)

        self.pi.write(self.SRL, 0)  # SR
        self.pi.write(self.SRH, 0)                # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def BEND_CLOCK_P(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0)               # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc) # -

        self.pi.write(self.NRL, 0)               # NR
        self.pi.write(self.NRH, 0) # -    

        # South wheels
        self.pi.write(self.SLL, 0)                # SL
        self.pi.set_PWM_dutycycle(self.SLH, self.pwm_dc) # -

        self.pi.write(self.SRL, 0)                # SR
        self.pi.write(self.SRH, 0)  # -

        # delay for distance (* .5 sec for ~5cm)
        if dt != None:
            time.sleep(dt)

            # halt motors
            self.HALT()

    def CENTER_CLOCK(self, dt=None):
        # North wheels
        self.pi.set_PWM_dutycycle(self.NLL, self.pwm_dc)   # NL
        self.pi.write(self.NLH, 0)                    # -

        self.pi.write(self.NRL, 0)                    # NR
        self.pi.set_PWM_dutycycle(self.NRH, self.pwm_dc)   # -
        
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

    def CENTER_CLOCK_P(self, dt=None):
        # North wheels
        self.pi.write(self.NLL, 0)   # NL
        self.pi.set_PWM_dutycycle(self.NLH, self.pwm_dc)                    # -

        self.pi.set_PWM_dutycycle(self.NRL, self.pwm_dc)                    # NR
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


