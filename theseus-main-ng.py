
import re, requests, time, serial, cv2, numpy as np, subprocess, json, wave, pyaudio
import evdev, socket, pynmea2, base64, mimetypes, threading

from RPLCD.gpio import CharLCD
import RPi.GPIO as GPIO

from bigdatacloud import BigDataCloudClient
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

SESSION_DIR = "/home/trojanzaro/DUMMY_DIR"
OPS_SESSION_ID = "ses_robot_ops_static_00000001"
VISION_SESSION_ID = "ses_robot_vision_static_0001"

AUTH = ("opencode", "12345678")

stop_event = Event()
#stop_button = Button(17, pull_up=True, bounce_time=0.2)

# ROBOT STATE

# ROBOT STATE
@dataclass
class RobotState:
    lock: Lock = field(default_factory=Lock)

    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    setpoint: np.ndarray = field(default_factory=lambda: np.zeros(4))
    measured: np.ndarray = field(default_factory=lambda: np.zeros(4))
    output: np.ndarray = field(default_factory=lambda: np.zeros(4))

    stop_requested: bool = False

    lat: float = 0.0
    lon: float = 0.0
    location_desc: str = ''

    led: LED = field(init=False)
    lcd: CharLCD = field(init=False)

    def __post_init__(self):
        self.led = LED(27)

        self.lcd = CharLCD(
            numbering_mode=GPIO.BCM,
            cols=16,
            rows=2,
            pin_rs=26,
            pin_e=19,
            pins_data=[13, 6, 5, 11],
            pin_rw=None
        )

    def get_lcd(self):
        return self.lcd

    def set_velocity(self, vx, vy, wz):
        with self.lock:
            self.velocity[:] = [vx, vy, wz]
            self.stop_requested = False

    def stop(self):
        with self.lock:
            self.velocity[:] = 0
            self.setpoint[:] = 0
            self.output[:] = 0
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

    def update_location(self, lat, lon):
        with self.lock:
            self.lat = lat
            self.lon = lon

    def update_location_desc(self):
        client = BigDataCloudClient(
            "api-key"
        )

        place = client.reverse_geocoding.reverse_geocode(
            self.lat, self.lon
        )

        with self.lock:
            self.location_desc = (
                f"{place.city}, "
                f"{place.principal_subdivision}, "
                f"{place.country_name}"
            )

    def get_location(self):
        with self.lock:
            return self.lat, self.lon

    def get_location_desc(self):
        with self.lock:
            return self.location_desc

# FRAMESTATE
class FrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame = None

    def update(self, frame):
        with self._lock:
            self._frame = frame.copy()

    def get_jpeg_base64(self, max_width=800, quality=80):
        with self._lock:
            if self._frame is None:
                return None
            frame = self._frame.copy()

        # Resize down for fast local Ollama inference
        h, w = frame.shape[:2]
        if w > max_width:
            scale = max_width / w
            frame = cv2.resize(frame, (max_width, int(h * scale)))

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("utf-8")

latest_frame = FrameBuffer()
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

            latest_frame.update(frame)  # <-- Zero disk I/O, ~0.1ms overhead

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

# ask opencode

def ask_opencode(user_text, include_camera=False):
    print(f"[DEBUG] ask_opencode | user_text={user_text!r} | include_camera={include_camera}")

    if include_camera:
        b64_img = latest_frame.get_jpeg_base64()
        print(f"[VISION DEBUG] b64_img len={len(b64_img) if b64_img else 0}")

        if b64_img:
            try:
                print(f"[VISION DEBUG] POST -> http://192.168.191.141:11434/v1")
                v_res = requests.post(
                    'http://192.168.191.141:11434/api/chat',
                    json={
                        "model": "qwen3-vl:latest",
                        "stream": False,
                        "think": False,
                        "messages": [
                            {
                                "role": "system",
                                "content": "You are a helpful robot looking through your camera. Describe or answer what you see in 1 or 2 short sentences. Use plain spoken English only. Never use markdown, lists, asterisks, or emojis."
                            },
                            {
                                "role": "user",
                                "content": user_text or "What do you see in front of you?",
                                "images": [b64_img]
                            }
                        ]
                    },
                    timeout=60
                )
                print(f"[VISION DEBUG] status={v_res.status_code}")
                if not v_res.ok:
                    print(f"[VISION DEBUG] error={v_res.text}")
                    v_res.raise_for_status()

                reply = v_res.json()["message"]["content"].strip()
                print(f"[VISION DEBUG] reply={reply!r}")
                return reply
            except Exception as e:
                print(f"[VISION DEBUG] EXCEPTION: {type(e).__name__}: {e}")
                raise

    payload = {
        "agent": "assistant",
        "model": {
            "providerID": "ollama",
            "id": "granite4.2:3b"
        },
        "text": user_text,
    }

    p_res = requests.post(
        f"{OPENCODE_URL}/api/session/{OPS_SESSION_ID}/prompt",
        params={"directory": SESSION_DIR},
        json=payload,
        auth=AUTH,
        timeout=30
    )
    p_res.raise_for_status()
    user_msg_id = p_res.json()["data"]["id"]

    deadline = time.time() + 60
    while time.time() < deadline:
        time.sleep(0.3)
        m_res = requests.get(
            f"{OPENCODE_URL}/api/session/{OPS_SESSION_ID}/message",
            params={"directory": SESSION_DIR},
            auth=AUTH,
            timeout=10
        )
        m_res.raise_for_status()
        msgs = m_res.json().get("data", [])

        user_msg = next((m for m in msgs if m.get("id") == user_msg_id), None)
        user_created = user_msg.get("time", {}).get("created", 0) if user_msg else 0
        turn_msgs = [
            m for m in msgs
            if m.get("id") != user_msg_id
            and m.get("time", {}).get("created", 0) >= user_created
        ]

        is_idle = any(m.get("type") == "idle" for m in turn_msgs)
        assistant_msgs = [m for m in turn_msgs if m.get("type") == "assistant"]

        for msg in sorted(assistant_msgs, key=lambda x: x.get("time", {}).get("created", 0), reverse=True):
            if msg.get("error"):
                raise RuntimeError(f"OpenCode assistant error: {msg['error']}")

            if msg.get("time", {}).get("completed") or is_idle:
                text_parts = [
                    block.get("text", "")
                    for block in msg.get("content", [])
                    if block.get("type") == "text" and block.get("text", "").strip()
                ]
                reply = " ".join(text_parts).strip()
                if reply:
                    reply = re.sub(r"[*#_`~]", "", reply)
                    return reply

        if is_idle:
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
                    elif "where are we" in text:
                        robot.update_location_desc()
                        generate_words("we are currently at, location, "+ robot.get_location_desc())
                        speak_mouth("123123")
                    elif any(phrase in text.lower() for phrase in ['what can you see', 'what do you see', 'what are we looking at', 'what are you looking at']):
                        reply = ''
                        try:
                            reply = ask_opencode(text, True)
                            generate_words(reply)
                            speak_mouth('123')
                        except Exception as e:
                            reply = "sorry, my eyes are a bit blurry"
                    else:
                        robot.get_lcd().clear()
                        robot.get_lcd().write_string(text)
                        reply = ''
                        try:
                            reply = ask_opencode(text)
                        except Exception as e:
                            print(e)
                            SESSION_ID = None  # Reset session if connection dropped
                            reply = "Sorry, I could not reach my brain right now."
                        generate_words(reply)
                        speak_mouth("123123")
                        robot.get_lcd().write_string(reply)
    except Exception as e:
        print("Speech recognition error:", e)
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

# ANDROID GNSS

def location_controller():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('127.0.0.1', 10110))

    with sock.makefile('r') as stream:
        for line in stream:
            if line.startswith('$'):
                try:
                    msg = pynmea2.parse(line)
                    if hasattr(msg, 'latitude') and msg.latitude:
                        robot.update_location(msg.latitude, msg.longitude)
                        #print(f"Lat: {msg.latitude}, Lon: {msg.longitude}, Sats: {getattr(msg, 'num_sats', 'N/A')}")
                except pynmea2.ParseError:
                    continue

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


        # Y button image


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
    robot.get_lcd().write_string("Hello! I am Theseus")

    stop_button = Button(17, pull_up=True, bounce_time=0.2)
    port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
    threads = [
        Thread(target=serial_reader, args=(port,), daemon=True),
        Thread(target=motor_controller, args=(port,), daemon=True),
        Thread(target=vision_controller, daemon=True),
        Thread(target=speech_recog_controller, daemon=True),
        Thread(target=HID_controller, daemon=True),
        Thread(target=location_controller, daemon=True)
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
        robot.get_lcd().clear()
        robot.get_lcd().write_string("Shutting down!")
        robot.get_lcd().cursor_pos = (1, 0)
        robot.get_lcd().write_string("Good bye <3")
                
        generate_words("shutting down, goodbye")
        speak_mouth('wer')

        time.sleep(1.5)

        robot.get_lcd().clear()
        GPIO.cleanup([26, 19, 13, 6, 5, 11, 27])
        
        try:

            robot.lcd.close(clear=True)
        
            robot.led.close()
        except Exception as e:
            pass
        robot.stop()
        stop_event.set()
        for t in threads:
            t.join(timeout=1)
        if port.is_open:
            send_motor_speeds(port, [0, 0, 0, 0])
            port.close()

if __name__ == "__main__":
    main()
