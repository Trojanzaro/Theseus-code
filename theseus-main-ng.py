
import re, requests, time, serial, cv2, numpy as np, subprocess, json, wave, pyaudio
import evdev
from evdev import InputDevice, ecodes

from babel.dates import format_datetime
from datetime import datetime
from gpiozero import LED, Button

from dataclasses import dataclass, field
from threading import Lock, Thread, Event
from vosk import Model, KaldiRecognizer

# CONFIG
DT, KP = 0.05, 5.2
WHEEL_RADIUS, LX, LY = 0.033, 0.65, 0.7
CENTER_DEADZONE, TURN_SPEED = 60, 0.01
CAMERA_INDEX, SERIAL_PORT, BAUDRATE = 0, "/dev/ttyS0", 115200
OPENCODE_URL = "http://192.168.191.141:4096"
SESSION_ID = None  # Reuses the same conversation session across commands
SESSION_DIR = '/home/trojanzaro/DUMMY_DIR'
AUTH = ("opencode", "12345678")

stop_event = Event()
#stop_button = Button(17, pull_up=True, bounce_time=0.2)

# ROBOT STATE
@dataclass
class RobotState:
    lock: Lock = field(default_factory=Lock)
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    setpoint: np.ndarray = field(default_factory=lambda: np.zeros(4))
    measured: np.ndarray = field(default_factory=lambda: np.zeros(4))
    output: np.ndarray = field(default_factory=lambda: np.zeros(4))
    stop_requested: bool = False
    
    led = LED(27)

    def set_velocity(self, vx, vy, wz):
        with self.lock:
            self.velocity[:] = [vx, vy, wz]
            self.stop_requested = False

    def stop(self):
        with self.lock:
            self.velocity[:] = self.setpoint[:] = self.output[:] = 0
            self.stop_requested = True

    def get_velocity(self):
        with self.lock:
            return self.velocity.copy()

    def update_measured(self, values):
        with self.lock:
            self.measured[:] = values

    def update_controller(self, setpoint):
        with self.lock:
            self.setpoint[:] = setpoint
            if self.stop_requested:
                self.output[:] = 0
            else:
                self.output += (self.setpoint - self.measured) * DT * KP
            return self.output.copy()
    def LED_state(self):
        self.led.toggle()



robot = RobotState()
IK = (1 / WHEEL_RADIUS) * np.array([
    [1, -1, -(LX + LY)], [1, 1, LX + LY],
    [1, 1, -(LX + LY)], [1, -1, LX + LY]
])

# SERIAL
def send_motor_speeds(port, speeds):
    port.write((" ".join(f"{v:.2f}" for v in speeds) + "\n").encode())

def serial_reader(port):
    while not stop_event.is_set():
        try:
            if port.in_waiting:
                values = np.fromstring(port.readline().decode().strip(), sep=" ")
                if len(values) == 4 and np.all(np.isfinite(values)):
                    robot.update_measured(values)
            else:
                stop_event.wait(0.005)
        except (ValueError, serial.SerialException) as e:
            print("Serial read error:", e)
            stop_event.wait(0.1)

# MOTOR CONTROL
def motor_controller(port):
    while not stop_event.is_set():
        start = time.monotonic()
        send_motor_speeds(port, robot.update_controller(IK @ robot.get_velocity()))
        stop_event.wait(max(0, DT - (time.monotonic() - start)))

# VISION
def vision_controller():
    cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cam.isOpened():
        raise RuntimeError("Could not open camera")

    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250),
        cv2.aruco.DetectorParameters()
    )

    K, D = None, np.array([-0.40, 0.08, 0, 0], dtype=np.float32)
    lost = False

    try:
        while not stop_event.is_set():
            ret, frame = cam.read()
            if not ret:
                robot.stop()
                continue

            h, w = frame.shape[:2]
            if K is None:
                K = np.array([[w, 0, w/2], [0, w, h/2], [0, 0, 1]], dtype=np.float32)

            frame = cv2.undistort(frame, K, D)
            corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

            if ids is None:
                if not lost:
                    #print("Marker lost!")
                    continue
                robot.stop()
                lost = True
                continue

            lost = False
            marker_x = np.mean(corners[0][0], axis=0)[0]
            error = w/2 - marker_x

            if abs(error) <= CENTER_DEADZONE:
                robot.stop()
            else:
                robot.set_velocity(0, 0, np.sign(error) * TURN_SPEED)

    finally:
        robot.stop()
        cam.release()

def ask_opencode(user_text):
    global SESSION_ID, SESSION_DIR

    # 1. Create a session once
    if not SESSION_ID:
        s_res = requests.post(
            f"{OPENCODE_URL}/api/session",
            auth=AUTH,
            json={"title": "Robot"},
            timeout=10
        )
        s_res.raise_for_status()
        s_data = s_res.json()["data"]
        SESSION_ID = s_data["id"]
        SESSION_DIR = s_data["location"]["directory"]

    # 2. Submit the prompt asynchronously and record our user message ID
    p_res = requests.post(
        f"{OPENCODE_URL}/api/session/{SESSION_ID}/prompt",
        params={"directory": SESSION_DIR},
        json={
            "agent": "assistant",
            "text": user_text
        },
        auth=AUTH,
        timeout=15
    )
    p_res.raise_for_status()
    user_msg_id = p_res.json()["data"]["id"]

    # 3. Poll the session messages until the assistant finishes replying (up to 60s)
    deadline = time.time() + 60
    while time.time() < deadline:
        time.sleep(0.5)
        #print('requesting! ', f'{OPENCODE_URL}/api/session/{SESSION_ID}/message')
        m_res = requests.get(
            f"{OPENCODE_URL}/api/session/{SESSION_ID}/message",
            params={"directory": SESSION_DIR},
            auth=AUTH,
            timeout=10
        )
        m_res.raise_for_status()
        msgs = m_res.json().get("data", [])

        # Collect messages newer than our user_msg_id (msgs are ordered newest -> oldest)
        turn_msgs = []
        for msg in msgs:
            if msg.get("id") == user_msg_id:
                break
            turn_msgs.append(msg)

        # Wait until OpenCode emits the 'idle' event for this turn
        is_idle = any(m.get("type") == "idle" for m in turn_msgs)
        if not is_idle:
            continue

        # Grab the latest completed assistant message in this turn and extract 'text' from 'content'
        for msg in turn_msgs:
            if msg.get("type") == "assistant" and msg.get("time", {}).get("completed"):
                text_parts = [
                    block.get("text", "")
                    for block in msg.get("content", [])
                    if block.get("type") == "text"
                ]
                reply = " ".join(text_parts).strip()
                if reply:
                    reply = re.sub(r"[*#_`~]", "", reply)
                    return reply

        raise RuntimeError(f"Turn finished idle without text reply: {turn_msgs}")

    raise TimeoutError("Timed out waiting for OpenCode assistant reply")

# SPEECH OUTPUT
def generate_words(text):
    subprocess.run(["espeak", text, "-g", "10", "-v", "en", "-a", "200",
                    "-w", "/home/pi/theseus_wav/mouth.wav"], check=True)

def speak_mouth(flag):
    filename = "/home/pi/theseus_wav/mouth.wav"
    if flag == "horn":
        filename = "/home/pi/theseus_wav/horn.wav"

    p = pyaudio.PyAudio()
    try:
        with wave.open(filename, "rb") as wav:
            stream = p.open(format=p.get_format_from_width(wav.getsampwidth()),
                            channels=wav.getnchannels(), rate=wav.getframerate(), output=True)
            try:
                while data := wav.readframes(8192):
                    stream.write(data)
            finally:
                stream.stop_stream()
                stream.close()
    finally:
        p.terminate()

# SPEECH RECOGNITION
def speech_recog_controller():
    #model = Model("vosk-model-el-gr-0.7")
    model = Model("vosk-model-small-en-us-0.15")

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=48000,
                    input=True, input_device_index=3, frames_per_buffer=8192)
    recognizer = KaldiRecognizer(model, 48000)
    print("Speech recognition started!")

    try:
        while not stop_event.is_set():
            data = stream.read(8192, exception_on_overflow=False)
            if recognizer.AcceptWaveform(data):
                text = json.loads(recognizer.Result()).get("text", "").strip()
                if text:
                    print(text)
                    if text.startswith(("walk", "strafe", "turn")):
                        if "forward" in text:
                            robot.set_velocity(0.1, 0, 0)
                            time.sleep(1)
                            robot.stop()
                        elif "backwards" in text:
                            robot.set_velocity(-0.1, 0, 0)
                            time.sleep(1)
                            robot.stop() 
                        elif "strafe" in text and "left" in text:
                            robot.set_velocity(0, 0.1, 0)
                            time.sleep(1)
                            robot.stop()
                        elif "strafe" in text and  "right" in text:
                            robot.set_velocity(0, -0.1, 0)
                            time.sleep(1)
                            robot.stop()
                    else:
                        reply = ''
                        try:
                            reply = ask_opencode(text)
                        except Exception as e:
                            print(e)
                            SESSION_ID = None  # Reset session if connection dropped
                            reply = "Sorry, I could not reach my brain right now."
                        generate_words(reply)
                        speak_mouth("123123")
    except Exception as e:
        print("Speech recognition error:", e)
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

MAX_VEL = 0.2    # Max linear speed (m/s) - adjust to match your robot's safe speed
MAX_OMEGA = 0.5  # Max rotation speed (rad/s)

# JOYSTICK INPUT
def HID_controller():
    dev = next((InputDevice(p) for p in evdev.list_devices() if "F710" in InputDevice(p).name or "Logitech" in InputDevice(p).name), None)
    if not dev: exit("F710 not found")
    print('Controller conected')

    state = {"lx": 0.0, "ly": 0.0, "rx": 0.0}

    AXES = {ecodes.ABS_X: "LX", ecodes.ABS_Y: "LY", ecodes.ABS_RX: "RX", ecodes.ABS_RY: "RY"}

    for event in dev.read_loop():
        # 1. 'B' Button Emergency / Idle Stop
        if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_B and event.value == 1:
            state = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
            robot.stop()
            print("STOPPED (B pressed)")
            continue

        # LIGHT
        if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_A and event.value == 1:
            robot.LED_state()
            continue

        # horn
        if event.type == ecodes.EV_KEY and event.code == ecodes.BTN_X and event.value == 1:
            speak_mouth("horn")


        if event.type == ecodes.EV_ABS and event.code in AXES:
            val = round(event.value / 32768.0, 2)
            if abs(val) < 0.1:  # Ignore center drift
                val = 0.0
            
            #print(f"{AXES[event.code]}: {val}")
            
            if event.code == ecodes.ABS_Y:      # Forward / Backward
                state["ly"] = -val
            elif event.code == ecodes.ABS_X:    # Strafe Left / Right
                state["lx"] = -val
            elif event.code == ecodes.ABS_RX:   # Rotate Left / Right
                state["rx"] = -val
            else:
                continue

            # Map normalized values (-1 to 1) to robot velocities
            vx = round(state["ly"] * MAX_VEL, 3)
            vy = round(state["lx"] * MAX_VEL, 3)
            wz = round(state["rx"] * MAX_OMEGA, 3)

            # Call robot.stop() when sticks are centered so motors fully disengage
            if vx == 0.0 and vy == 0.0 and wz == 0.0:
                robot.stop()
            else:
                robot.set_velocity(vx, vy, wz)


# MAIN
def main():
    stop_button = Button(17, pull_up=True, bounce_time=0.2)
    port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
    threads = [
        Thread(target=serial_reader, args=(port,), daemon=True),
        Thread(target=motor_controller, args=(port,), daemon=True),
        Thread(target=vision_controller, daemon=True),
        Thread(target=speech_recog_controller, daemon=True),
        Thread(target=HID_controller, daemon=True)
    ]

    try:
        for t in threads:
            t.start()
        generate_words("Good evening! My name is Theseus, Awaiting orders!")
        speak_mouth("asdasd")
        while any(t.is_alive() for t in threads):
            if stop_button.is_pressed:
                print("\nStop button pressed! Stopping robot...")
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Stopping robot...")
    finally:
        generate_words("shutting down, goodbye")
        speak_mouth('wer')

        robot.stop()
        stop_event.set()
        for t in threads:
            t.join(timeout=1)
        if port.is_open:
            send_motor_speeds(port, [0, 0, 0, 0])
            port.close()

if __name__ == "__main__":
    main()
