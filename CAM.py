import cv2
import serial
import time
import numpy as np

# Initialize serial connection to Arduino
arduino = serial.Serial('COM4', 9600)  # Update with your Arduino port
time.sleep(2)  # Wait for the connection to initialize

# Load Haar cascades for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initial pan and tilt angles
x_angle = 270 # Pan (controlled by stepper motor)
y_angle = 90  # Tilt (controlled by servo motor)

# Define angle boundaries for pan and tilt
x_min, x_max = 0, 540  
y_min, y_max = 40, 140

# Initialize Kalman Filter
def initialize_kalman_filter():
    kalman = cv2.KalmanFilter(4, 2)  # 4 dynamic params (x, y, velocity_x, velocity_y), 2 measurements (x, y)
    kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                         [0, 1, 0, 0]], np.float32)
    kalman.transitionMatrix = np.array([[1, 0, 1, 0],
                                        [0, 1, 0, 1],
                                        [0, 0, 1, 0],
                                        [0, 0, 0, 1]], np.float32)
    kalman.processNoiseCov = np.array([[1, 0, 0, 0],
                                       [0, 1, 0, 0],
                                       [0, 0, 5, 0],
                                       [0, 0, 0, 5]], np.float32) * 0.03
    return kalman

kalman_filter = initialize_kalman_filter()

# Adjust motor positions based on object location
def update_motor_position(x, y):
    global x_angle, y_angle

    # Adjust x angle for the stepper motor (pan)
    if x < 240:  # Left of center
        x_angle = min(x_max, x_angle + 1)
        
    elif x > 400:  # Right of center
        x_angle = max(x_min, x_angle - 1)
    
    # Inverted logic for tilt servo
    if y < 160:  # Above center (tilt up)
        y_angle = min(y_max, y_angle + 2)  # Increase tilt angle (tilt up)
    elif y > 320:  # Below center (tilt down)
        y_angle = max(y_min, y_angle - 2)  # Decrease tilt angle (tilt down)

    # Send commands to Arduino
    print(f'X{x_angle}Y{y_angle}\n')
    arduino.write(f'X{x_angle}Y{y_angle}\n'.encode('utf-8'))
    time.sleep(0.05)  # Small delay to allow Arduino to respond

# Start video capture from external webcam (index 1)
cap = cv2.VideoCapture(1,cv2.CAP_DSHOW)
cap.set(3, 640)  # Set frame width to 640
cap.set(4, 480)  # Set frame height to 480

# Frame size
frame_width = 640
frame_height = 480

# Simulation boundaries
stepper_min = 0
stepper_max = frame_width
servo_min = 0
servo_max = frame_height

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Flip the frame horizontally to mirror the camera
    frame = cv2.flip(frame, 1)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 5)

    # Detect face and adjust motors to center face in frame
    if len(faces) > 0:
        x, y, w, h = faces[0]  # Take the first detected face
        center_x, center_y = x + w // 2, y + h // 2

        # Update the Kalman filter with the new measurements (detected center)
        measurement = np.array([[np.float32(center_x)], [np.float32(center_y)]])
        kalman_filter.correct(measurement)

        # Predict the next position
        prediction = kalman_filter.predict()
        pred_x, pred_y = int(prediction[0]), int(prediction[1])

        # Draw detection and prediction
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.circle(frame, (pred_x, pred_y), 5, (0, 255, 0), -1)  # Prediction dot

        # Display the coordinates
        cv2.putText(frame, f"Actual: ({center_x}, {center_y})", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Predicted: ({pred_x}, {pred_y})", (x, y - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Simulate stepper motor (horizontal movement) for X-axis
        stepper_position = int(np.clip(pred_x, stepper_min, stepper_max))
        cv2.line(frame, (stepper_position, 0), (stepper_position, frame_height), (0, 0, 255), 2)  # Simulated motor line

        # Simulate servo motor (vertical movement) for Y-axis
        servo_angle = int(np.clip(pred_y, servo_min, servo_max))
        cv2.line(frame, (0, servo_angle), (frame_width, servo_angle), (0, 255, 255), 2)  # Simulated servo line

        # Simulated "Motor Positions" Display
        cv2.putText(frame, f"Stepper Pos: {stepper_position}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(frame, f"Servo Angle: {servo_angle}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Send motor updates to Arduino based on actual center
        update_motor_position(center_x, center_y)

    # Display the resulting frame
    cv2.imshow('Face Tracking with Visual Feedback', frame)

    # Exit loop on 'ESC' key press
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
cap.release()
cv2.destroyAllWindows()
arduino.close()
