class Stage:
    """A class that will allow the robot to move in a 2D stage"""
    def __init__(self, motors):

        """The stage is defined as a 2D array"""
        self.stage = [
            [0, 0, 0, 0, 0, 0], 
            [0, 0, 0, 0, 0, 0], 
            [0, 0, '>', '>', 'v', 0], 
            [0, 0, '^', 0, 'v', 0],
            [0, 0, '^', 0, 'v', 0],
            [0, 0, '^', '<', '<', 0], 
            [0, 0, 0, 0, 0, 0], 
        ]

        """The motors object to control the motors"""
        self.motors = motors

        """The distance the robot will travel per element"""
        self.dist = 0.5 # 0.5 sec  = ~5 cm

        """A kernel of guidance is needed to read 'where it is' and 'where it's going'"""
        self.kernel = {}
        self.kernel['r'] = 5
        self.kernel['c'] = 2
    
    """Print the entire stage for debuging pursposes"""
    def print_stage(self):
        rows = len(self.stage)
        cols = len(self.stage[0])

        for i in range(rows):
            for j in range(cols):
                print(self.stage[i][j],  end=" ")
            print()

    def kernel_ctr(self):
        k = self.stage[self.kernel['r']][self.kernel['c']]

        if k == '^':
            print(k, "FORWARD")
            self.kernel['r'] -= 1
            self.motors.NORTH(self.dist)

        if k == '>':
            print(k, "RIGHT")
            self.kernel['c'] += 1
            self.motors.STRAFE_RIGHT(self.dist)

        if k == 'v':
            print(k, "BACKWARD")
            self.kernel['r'] += 1
            self.motors.SOUTH(self.dist)

        if k == '<':
            print(k, "LEFT")
            self.kernel['c'] -= 1
            self.motors.STRAFE_LEFT(self.dist)
        