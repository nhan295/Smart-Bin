import time
import cv2
import RPi.GPIO as GPIO
from picamera2 import Picamera2
from ultralytics import YOLO

# Set up GPIO pins for servo motors
servo1_pin = 16  # Servo 1 connected to GPIO 16
servo2_pin = 27  # Servo 2 connected to GPIO 27

GPIO.setmode(GPIO.BOARD)  # Use BOARD pin numbering

# Initialize GPIO pins
GPIO.setup(servo1_pin, GPIO.OUT)
GPIO.setup(servo2_pin, GPIO.OUT)

# Initialize PWM for servos with a 50Hz frequency
servo1 = GPIO.PWM(servo1_pin, 50)
servo2 = GPIO.PWM(servo2_pin, 50)
servo1.start(0)  # Initial position
servo2.start(0)

# Define function to rotate servo to a specific angle
def rotate_servo(servo, target_angle, step=1, delay=0.1):
    current_angle = 0
    while current_angle < target_angle:
        duty_cycle = 2 + (current_angle / 18)  # Calculate duty cycle
        servo.ChangeDutyCycle(duty_cycle)
        time.sleep(delay)
        current_angle += step  # Increment angle
    servo.ChangeDutyCycle(0)  # Stop signal after rotation

# Define function to handle servo, rotate and return to initial position
def handle_servo(servo, angle=180, delay=4):
    rotate_servo(servo, angle)  # Rotate to target angle
    time.sleep(delay)  # Hold position for specified time
    # Rotate back to the initial position
    rotate_servo(servo, 0)

# Initialize Picamera2 and YOLO model
picam2 = Picamera2()
model = YOLO("best14_10.pt")

# Start camera
picam2.start()

try:
    while True:
        # Capture image from the camera
        try:
            image = picam2.capture_array()
        except Exception as e:
            print(f"Error capturing image: {e}")
            continue

        # Convert 4-channel image to RGB if needed
        if image.shape[2] == 4:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        else:
            image_rgb = image

        # Resize image to improve model performance
        image_rgb_resized = cv2.resize(image_rgb, (320, 320))

        # Make predictions using the YOLO model
        results = model(image_rgb_resized)

        # Process each detected object
        for result in results:
            for obj in result.boxes:
                cls_id = int(obj.cls[0])  # Get class ID
                label = model.names[cls_id]  # Get label from class ID

                if label == 'khong tai che':  # Non-recyclable
                    print("Detected: Non-recyclable")
                    handle_servo(servo1)  # Activate servo 1
                elif label == 'tai che':  # Recyclable
                    print("Detected: Recyclable")
                    handle_servo(servo2)  # Activate servo 2

                time.sleep(1)  # Wait a bit before the next detection

except KeyboardInterrupt:
    print("Program interrupted...")

finally:
    # Stop servos and clean up GPIO
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()
    picam2.stop()
