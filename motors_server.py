import pigpio
from flask import Flask
import motors_ctr

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
# interuopt GPIO forstopping by callback cb_stop_bt 
cb1 = pi1.callback(4, pigpio.EITHER_EDGE, cb_stop_bt)

# Initialize the motors GPIOs 
motors = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6)

####
def create_motors_server(motors_instance):
        app = Flask(__name__)
        app.config['MOTORS'] = motors_instance
        return app

app = create_motors_server(motors)

@app.route("/")
def index(): 
     return "<h1>I'M ANGERY</h1>"

@app.route('/north')
def north():
    app.config['MOTORS'].NORTH()
    print("YO I'M HERE!!!")
    return "<h1>hello</h1>"

@app.route('/south')
def south():
    app.config['MOTORS'].SOUTH()
    print("YO I'M HERE!!!")
    return "<h1>hello</h1>"


@app.route('/left')
def left():
    app.config['MOTORS'].STRAFE_LEFT()
    print("YO I'M HERE!!!")
    return "<h1>hello</h1>"

@app.route('/right')
def right():
    app.config['MOTORS'].STRAFE_RIGHT()
    print("YO I'M HERE!!!")
    return "<h1>hello</h1>"

@app.route('/halt')
def halt():
    app.config['MOTORS'].HALT()
    print("YO I'M HERE!!!")
    return "<h1>hello</h1>"
####

####
# MAIN LOOP
####
if __name__ == '__main__':
    app.run(host="0.0.0.0")

