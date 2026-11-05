import time
import serial
import cv2
import numpy as np

from dataclasses import dataclass, field
from threading import Lock, Thread, Event


# ROBOT STATE

@dataclass
class RobotState:
    lock: Lock = field(default_factory=Lock)

    # Desired robot velocity: [Vx, Vy, Wz]
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )

    # Desired and measured motor velocities: [w1, w2, w3, w4]
    setpoint: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )
    measured: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )
    output: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )

    def set_velocity(self, vx, vy, wz):
        with self.lock:
            self.velocity[:] = [vx, vy, wz]

    def update_measured(self, velocities):
        with self.lock:
            self.measured[:] = velocities

    def get_velocity(self):
        with self.lock:
            return self.velocity.copy()

    def get_measured(self):
        with self.lock:
            return self.measured.copy()

    def update_controller(self, setpoint, dt, kp):
        with self.lock:
            self.setpoint[:] = setpoint

            error = self.setpoint - self.measured
            self.output += error * dt * kp

            return self.output.copy()


# CONFIGURATION

DT = 0.05
KP = 5.2

WHEEL_RADIUS = 0.033
LX = 0.65
LY = 0.7

CENTER_DEADZONE = 60
TURN_SPEED = 0.05

CAMERA_INDEX = 0
SERIAL_PORT = "/dev/ttyS0"
BAUDRATE = 115200

robot = RobotState()
stop_event = Event()


# INVERSE KINEMATICS

TRANSFORM_MATRIX = (1 / WHEEL_RADIUS) * np.array([
    [1, -1, -(LX + LY)],
    [1,  1,  (LX + LY)],
    [1,  1, -(LX + LY)],
    [1, -1,  (LX + LY)]
])


# SERIAL COMMUNICATION

def send_motor_speeds(port, speeds):
    command = " ".join(f"{speed:.2f}" for speed in speeds)
    port.write((command + "\n").encode())


def serial_reader(port):
    while not stop_event.is_set():
        try:
            if port.in_waiting == 0:
                time.sleep(0.005)
                continue

            response = port.readline().decode().strip()
            velocities = np.fromstring(response, sep=" ")

            if len(velocities) == 4 and np.all(np.isfinite(velocities)):
                robot.update_measured(velocities)

        except (ValueError, serial.SerialException) as ex:
            print("Serial read error:", ex)
            time.sleep(0.1)


# MOTOR CONTROLLER

def motor_controller(port):
    while not stop_event.is_set():
        start = time.monotonic()

        # Desired robot velocity
        velocity = robot.get_velocity()

        # Inverse kinematics
        setpoint = TRANSFORM_MATRIX @ velocity

        # Feedback controller
        output = robot.update_controller(setpoint, DT, KP)

        # Send motor commands
        send_motor_speeds(port, output)

        # Maintain a 50 ms control period
        elapsed = time.monotonic() - start
        stop_event.wait(max(0, DT - elapsed))


# VISION CONTROLLER

def vision_controller():
    cam = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    if not cam.isOpened():
        raise RuntimeError("Could not open camera")

    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))

    aruco_dict = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_6X6_250
    )
    detector = cv2.aruco.ArucoDetector(
        aruco_dict,
        cv2.aruco.DetectorParameters()
    )

    try:
        while not stop_event.is_set():
            ret, frame = cam.read()

            if not ret:
                robot.set_velocity(0, 0, 0)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)

            # No marker: stop the robot
            if ids is None:
                robot.set_velocity(0, 0, 0)
                continue

            # Use the first detected marker
            marker_x = corners[0][0][0][0]
            distance = (frame_width / 2) - marker_x

            # Centered: stop
            if abs(distance) <= CENTER_DEADZONE:
                robot.set_velocity(0, 0, 0)

            # Marker to the right: turn right
            elif distance < 0:
                robot.set_velocity(0, 0, -TURN_SPEED)

            # Marker to the left: turn left
            else:
                robot.set_velocity(0, 0, TURN_SPEED)

    finally:
        cam.release()


# MAIN

def main():
    port = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUDRATE,
        timeout=0.1,
        write_timeout=0.1
    )

    threads = []

    try:
        threads = [
            Thread(target=serial_reader, args=(port,), daemon=True),
            Thread(target=motor_controller, args=(port,), daemon=True),
            Thread(target=vision_controller, daemon=True)
        ]

        for thread in threads:
            thread.start()

        # Keep the main thread alive
        while any(thread.is_alive() for thread in threads):
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Stopping robot...")

    finally:
        stop_event.set()

        for thread in threads:
            thread.join(timeout=1)

        if port.is_open:
            send_motor_speeds(port, [0, 0, 0, 0])
            port.close()


if __name__ == "__main__":
    main()
