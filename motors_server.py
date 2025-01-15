import pigpio
from flask import Flask, g
import motors_ctr

# setup PIGPIO
pi1 = pigpio.pi()       # pi1 accesses the local Pi's GPIO

####
#callback function for terminating code
def cb_stop_bt(gpio, level, tick):
    print(gpio, level, tick)
    print("STOPPING APPLICATION")
    get_motors_instance().HALT()
    pi1.stop()
    exit()
# interuopt GPIO forstopping by callback cb_stop_bt 
cb1 = pi1.callback(4, pigpio.EITHER_EDGE, cb_stop_bt)

# Initialize the motors GPIOs 
# FLASK GLOBAL OBJECT REQUIRED! in order to store motors_instance in the global namespace ('g')
def get_motors_instance():
    if 'mot_ins' not in g:
        g.mot_ins = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6)
    return g.mot_ins

####
def create_motors_server():
        app = Flask(__name__)
        return app

app = create_motors_server()

@app.route("/")
def index(): 
     return "<h1>I'M ANGERY</h1>"

@app.route('/north')
def north():
    get_motors_instance().NORTH()
    return "<h1>hello</h1>"

@app.route('/south')
def south():
    get_motors_instance().SOUTH()
    return "<h1>hello</h1>"

@app.route('/left')
def left():
    get_motors_instance().STRAFE_LEFT()
    return "<h1>hello</h1>"

@app.route('/right')
def right():
    get_motors_instance().STRAFE_RIGHT()
    return "<h1>hello</h1>"

@app.route('/clock')
def clock():
    get_motors_instance().CLOCK()
    return "<h1>hello</h1>"

@app.route('/clock_p')
def clock_p():
    get_motors_instance().CLOCK_P()
    return "<h1>hello</h1>"

@app.route('/halt')
def halt():
    get_motors_instance().HALT()
    return "<h1>hello</h1>"
####

####
# MAIN LOOP
####
if __name__ == '__main__':
    app.run(host="0.0.0.0")

