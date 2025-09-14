import pigpio
from flask import Flask, g, request, current_app, render_template
import motors_ctr
import subprocess
import sonar_trigger_echo
import time
import json
import struct
import math
import string

import pyaudio
from vosk import Model, KaldiRecognizer
import wave

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
    if 'motors_instance' not in current_app.config:
        current_app.config['motors_instance'] = motors_ctr.Motors(pi1, 17, 27, 22, 10, 9, 11, 5, 6)
    return current_app.config['motors_instance']

def get_sonar_instance():
    if 'sonar_instance' not in current_app.config:
        current_app.config['sonar_instance'] = sonar_trigger_echo.ranger(pi1, 23, 18)
    return current_app.config['sonar_instance']

def generate_words(text):
    cmd = ["espeak", "\"{}\"".format(text), "-g", "10", "-v", "el", "-a", "200", "-w", "/home/pi/theseus_wav/mouth.wav"]
    subprocess.call(cmd)

####
# SPEECH TOKENS
def execute_tokens(text=""):
    tokens = text.split(" ")
    s1_tk = ['say', 'walk', 'strafe', 'turn', 'shut']
    if tokens[0] in s1_tk:
        if tokens[0] == "shut":
            if tokens[1] == "down":
                cmd = ["sudo", "shutdown", "now"]
                subprocess.call(cmd)

        if tokens[0] == 'say':
            text = ' '.join(tokens[1:])
            generate_words(text)
            speak_mouth()
        
        if tokens[0] == 'walk':
            if tokens[1] in ['north', 'front', 'forward', 'straight', 'ahead']:
                get_motors_instance().NORTH(1)
            if tokens[1] in ['south', 'backwards', 'back']:
                get_motors_instance().SOUTH(1)
        
        if tokens[0] == 'turn':
            if tokens[1] in ['clockwise', 'right']:
                get_motors_instance().CLOCK(1)
            if tokens[1] in ['left', 'counter']:
                get_motors_instance().CLOCK_P(1)
    else:
        get_motors_instance().CLOCK(0.5)
        get_motors_instance().CLOCK_P(0.5)

# GET RMS TO DETECT WHEN NO LONGER TALKING 
def get_rms( block ):
    SHORT_NORMALIZE = (1.0/32768.0)
    # RMS amplitude is defined as the square root of the 
    # mean over time of the square of the amplitude.
    # so we need to convert this string of bytes into 
    # a string of 16-bit samples...

    # we will get one short out for each 
    # two chars in the string.
    count = len(block)/2
    format = "%dh"%(count)
    shorts = struct.unpack( format, block )

    # iterate over the block.
    sum_squares = 0.0
    for sample in shorts:
        # sample is a signed short in +/- 32768. 
        # normalize it to 1.0
        n = sample * SHORT_NORMALIZE
        sum_squares += n*n

    return math.sqrt( sum_squares / count )

def speak_mouth():
    chunk_size = 8192
    p = pyaudio.PyAudio()
    wfr = wave.open("/home/pi/theseus_wav/mouth.wav", 'rb') if request.args.get("text") != "do_not_rr" else wave.open("/home/pi/theseus_wav/rr.wav", 'rb') # !
    stream = p.open(format=p.get_format_from_width(wfr.getsampwidth()),
                    channels=wfr.getnchannels(),
                    rate=wfr.getframerate(),
                    output=True)
    
    while len(data := wfr.readframes(chunk_size)):  # Requires Python 3.8+ for :=
        stream.write(data)
    stream.close()

####
def create_motors_server():
        app = Flask(__name__, template_folder='./')
        return app

app = create_motors_server()

@app.route("/")
def index(): 
    return render_template('index.html')

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

@app.route('/change_dc')
def change_dc():
    get_motors_instance().change_dc(request.args.get("dc"))
    return "<h1>hello</h1>"

@app.route('/get_distance')
def get_distance():
    distance = (get_sonar_instance().read() / 2) / 29.1
    return str(distance)

####
# SPEECH RECOGNITION

@app.route('/listen')
def listen_mic():
    model_path = "SpeechRecognition/models/vosk-model-small-en-us-0.15/"
    model = Model(model_path)

    # Settings for PyAudio
    sample_rate = 16000
    chunk_size = 8192
    format = pyaudio.paInt16
    channels = 1
    RECORD_SECONDS = 5

    ####
    # PADDING MOVEMENTS!
    get_motors_instance().NORTH(0.5)
    get_motors_instance().SOUTH(0.5)
    time.sleep(0.5)
    #
    ####

    # Initialization of PyAudio and speech recognition
    p = pyaudio.PyAudio()
    stream = p.open(format=format, channels=channels, rate=sample_rate, input=True, frames_per_buffer=chunk_size)
    recognizer = KaldiRecognizer(model, sample_rate)

   
    frames = []
    while True:
        data = stream.read(chunk_size)
        frames.append(data)
        amplitude = get_rms(data)
        if(amplitude <= 0.007): break # silence 

    stream.stop_stream()
    stream.close()
    #p.terminate()

    wf = wave.open("/home/pi/theseus_wav/ears.wav", 'wb')
    wf.setnchannels(channels)
    wf.setsampwidth(p.get_sample_size(format))
    wf.setframerate(sample_rate)
    wf.writeframes(b''.join(frames))
    wf.close()
    
    text_lst = []
    p_text_lst = []
    p_str = []
    wfr = wave.open("/home/pi/theseus_wav/ears.wav", 'rb')
    while True:
        data = wfr.readframes(chunk_size)
        if len(data) == 0:
            break
        if recognizer.AcceptWaveform(data):
            text_lst.append(recognizer.Result())
        else:
            p_text_lst.append(recognizer.PartialResult())
    wfr.close()

    if len(text_lst) !=0:
        jd = json.loads(text_lst[0])
        txt_str = jd["text"]
        
    elif len(p_text_lst) !=0: 
        for i in range(0,len(p_text_lst)):
            temp_txt_dict = json.loads(p_text_lst[i])
            p_str.append(temp_txt_dict['partial'])
       
        len_p_str = [len(p_str[j]) for j in range(0,len(p_str))]
        max_val = max(len_p_str)
        indx = len_p_str.index(max_val)
        txt_str = p_str[indx]
    else:
        txt_str =''

    print(txt_str)
    execute_tokens(txt_str)

    return "<h1>hello</h1>"

####
# ESPEAK URL
####
@app.route('/speak')
def speak():
    generate_words(request.args.get('text'))
    get_motors_instance().CLOCK_P(0.5)
    speak_mouth()
    return "<h1>hello</h1>"
####

####
# MAIN LOOP
####
if __name__ == '__main__':
    app.run(host="0.0.0.0", debug=True)

