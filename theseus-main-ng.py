
import requests, time, serial, cv2, numpy as np, subprocess, json, wave, pyaudio

from babel.dates import format_datetime
from datetime import datetime
from gpiozero import Button

from dataclasses import dataclass, field
from threading import Lock, Thread, Event
from vosk import Model, KaldiRecognizer

# CONFIG
DT, KP = 0.05, 5.2
WHEEL_RADIUS, LX, LY = 0.033, 0.65, 0.7
CENTER_DEADZONE, TURN_SPEED = 60, 0.01
CAMERA_INDEX, SERIAL_PORT, BAUDRATE = 0, "/dev/ttyS0", 115200

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

# SPEECH OUTPUT
def generate_words(text):
    subprocess.run(["espeak", text, "-g", "10", "-v", "en", "-a", "200",
                    "-w", "/home/pi/theseus_wav/mouth.wav"], check=True)

def speak_mouth(flag):
    filename = "/home/pi/theseus_wav/mouth.wav"
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
                        res = requests.post("http://192.168.191.141:11434/api/chat", json={
                            "model": "llama3.2:3b",
                            "messages": [
                                {"role": "system", "content": "You are a helpful robot. Reply in 1 or 2 short sentences. Use plain spoken English only. Never use markdown, lists, asterisks, or emojis."},
                                {"role": "user", "content": text}
                            ],
                            "options": {"num_predict": 50, "temperature": 0.3},
                            "stream": False
                        })
                        reply = res.json()["message"]["content"]
                        generate_words(reply)
                        speak_mouth("123123")




    except Exception as e:
        print("Speech recognition error:", e)
    finally:
        stream.stop_stream()
        stream.close()
        p.terminate()

# MAIN
def main():
    stop_button = Button(17, pull_up=True, bounce_time=0.2)
    port = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1, write_timeout=0.1)
    threads = [
        Thread(target=serial_reader, args=(port,), daemon=True),
        Thread(target=motor_controller, args=(port,), daemon=True),
        Thread(target=vision_controller, daemon=True),
        Thread(target=speech_recog_controller, daemon=True)
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
        robot.stop()
        stop_event.set()
        for t in threads:
            t.join(timeout=1)
        if port.is_open:
            send_motor_speeds(port, [0, 0, 0, 0])
            port.close()

if __name__ == "__main__":
    main()

