# Rotary wheel encoder and BNO055 IMU-powered navigation system on Raspberry Pi 5 for distance and angle tracking

# Import libraries
import time
import adafruit_bno055
import busio
import board
from math import pi
from gpiozero import Button
from gpiozero import DistanceSensor
from lidar import detect_object_of_interest
from classification import run_ml_pipeline
from ultrasonic import is_tall_object_present

## Rotary wheel encoder setup ##

# Declare GPIO pins and channels
left_A = 4  # Left encoder, channel A, GPIO pin 4 (corresponds to pin 7)
left_B = 17
right_A = 27
right_B = 22

# Encoder parameters
ppr = 12
decoding_factor = 2  # Pulse on rising and falling edges for 2x decoding
eff_cpr = ppr * decoding_factor
r = 10  # 10 cm wheel radius for testing purposes - measure later
cm_per_pulse = 2 * pi * r / eff_cpr

# State
left_count = 0  # Directional net count
right_count = 0
left_distance = 0.0  # Total distance regardless of direction
right_distance = 0.0
start_time_encoder = time.time()

# Noise rejection
min_pulse_interval = 0.001  # Max pulse frequency @ 1000 Hz
last_left_time = 0
last_right_time = 0

# Channel B input, disable internal pullup resistors as we have an external circuit
left_B = Button(left_B, pull_up=False)
right_B = Button(right_B, pull_up=False)

# Interrupt handlers for Channel A
def on_left_A():
    global left_count, last_left_time
    now = time.time()
    if now - last_left_time >= min_pulse_interval:
        direction = -1 if left_B.is_pressed else 1
        left_count += direction
        last_left_time = now

def on_right_A():
    global right_count, last_right_time
    now = time.time()
    if now - last_right_time >= min_pulse_interval:
        direction = -1 if right_B.is_pressed else 1
        right_count += direction
        last_right_time = now

# Set up channel A with rising (released) and falling (pressed) edges
left_button = Button(left_A, pull_up=False)
right_button = Button(right_A, pull_up=False)
left_button.when_pressed = on_left_A
left_button.when_released = on_left_A
right_button.when_pressed = on_right_A
right_button.when_released = on_right_A

## BNO055 IMU setup ##

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# State (z axis only for 2D angle tracking)
filtered_gyro = 0.0  # Filtered angular velocity
angle = 0.0

# Filtering
alpha = 0.1  # Low pass filter
gyro_deadzone = 0.2  # High pass filter

# Timer
prev_time_imu = time.time()
start_time_imu = prev_time_imu

## Ultrasonic sensor setup ##
sensor = DistanceSensor(echo=24, trigger=23, max_distance=2.5)
def is_tall_object_present(lidar_distance_mm, tolerance_mm=150):
    # Returns true if the ultrasonic sensor detects an object near the LiDAR
    measured_mm = sensor.distance * 1000
    print(f"Ultrasonic Sensor measured distance: {measured_mm:.0f} mm")

    if abs(measured_mm - lidar_distance_mm) <= tolerance_mm:
        return True
    return False

## Navigation variables ##
path_distance = 0.0         # Distance travelled on preset path (reset each turn)
path_angle = 0.0            # Accumulated angle on path (reset and adjust at 10 degrees)
garbage_distance = 0        # Distance travelled when collecting garbage (reset after collection)
garbage_angle = 0.0         # Adjusted angle when collecting garbage (reset after collection)

## Garbage distance tracking
garbage_distance_flag = False
prev_left_garbage = 0
prev_right_garbage = 0

def start_garbage_distance():
    global garbage_distance_flag, prev_left_garbage, prev_right_garbage
    garbage_distance_flag = True
    prev_left_garbage = left_count
    prev_right_garbage = right_count
    
def update_garbage_distance(): # Update garbage distance with new pulses
    global garbage_distance, prev_left_garbage, prev_right_garbage
    if garbage_distance_flag:
        delta_left_count = left_count - prev_left_garbage
        delta_right_count = right_count - prev_right_garbage
        delta_avg_count = (delta_left_count + delta_right_count) / 2
        garbage_distance += delta_avg_count * cm_per_pulse
        prev_left_garbage = left_count
        prev_right_garbage = right_count
        
    return garbage_distance
        
def reset_garbage_distance():
    global garbage_distance_flag, garbage_distance
    garbage_distance_flag = False
    garbage_distance = 0.0

## Garbage angle tracking
garbage_angle_flag = False
initial_garbage_angle = 0.0

def start_garbage_angle():
    global garbage_angle_flag, initial_garbage_angle
    garbage_angle_flag = True
    initial_garbage_angle = angle
    
def update_garbage_angle():
    global garbage_angle
    if garbage_angle_flag:
        gyro = sensor.gyro
        if gyro is not None:
            raw_gyro = gyro[2] * 180 / pi
            filtered_gyro = alpha * raw_gyro + (1 - alpha) * filtered_gyro  # Low pass filter
            if abs(filtered_gyro) < gyro_deadzone: filtered_gyro = 0        # High pass filter
            
            # Integrate angular velocity to calculate angle
            garbage_angle += filtered_gyro * dt
            # Normalized angle difference
            garbage_angle = (garbage_angle - initial_garbage_angle) % 360
            
    return garbage_angle
        
def reset_garbage_angle():
    global garbage_angle_flag
    garbage_angle_flag = False

## Path distance tracking

## Path angle tracking