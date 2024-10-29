### 1. Description
LockTron is a sophisticated real-time target-locking system, designed to detect, track, and predict dynamic target movements with minimal latency and high precision. The system adapts in real-time to variations in speed, direction, and acceleration, ensuring continuous lock on a moving target.

### 2. Construction
LockTron is built on a sturdy wooden platform with reinforced aluminum channels for stability. The setup includes:

Base Platform and Disk: A wooden base (300x300x18 mm) supports the structure, while a 15 mm diameter wooden disk mounts the motors.
Motor Mounting: A stepper motor controls x-axis rotation, and a servo motor manages y-axis adjustments, both mounted on the disk using a flange coupler.
Support Components: Channels and clamps secure the webcam and maintain alignment, minimizing vibration.

### 3. Working 
Initialization: Establishes serial communication with Arduino, loads a Haar Cascade for face detection, and defines motor angle boundaries.

Face Detection & Prediction: Captures video frames to detect faces, calculates the center coordinates, and uses a Kalman Filter to predict the target's next position.

Motor Adjustment: Based on detected and predicted positions, updates pan (x-axis) and tilt (y-axis) angles, sending commands to Arduino for continuous target tracking.

Visual Feedback: Displays live video with visual markers for actual and predicted positions, helping ensure smooth tracking and lock-on.
### 4. Technology Used
OpenCV (Haar Cascade): For face recognition and real-time tracking.
Microcontroller Programming: To manage the dual motor setup, processing x and y coordinate inputs to control motor adjustments.

### 5. Language Used
Python: For implementing face recognition and OpenCV functions.
e=Embedded C (Arduino IDE): For microcontroller programming and motor control.

### 6. Components

- Arduino Nano: Controls sensors and motors.
- DRV8825 Driver: Controls stepper motor.
- Stepper Motor (Nema 17): Y-axis movement.
- Servo Motor (MG966R): X-axis movement.
- 16x2 I2C LCD: Displays target coordinates.
- MP1584 Buck Converter: Regulates voltage.(12v - 5v)
- Self-made Relay Module: Safe switching and spike protection.
- LEDs: Indicate current flow.
