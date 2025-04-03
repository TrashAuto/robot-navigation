# Import libraries
import time
import adafruit_bno055
import busio
import board
from math import pi
from gpiozero import Button

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
imu_sensor = adafruit_bno055.BNO055_I2C(i2c)

# State (z axis only for 2D angle tracking)
filtered_gyro = 0.0  # Filtered angular velocity
angle = 0.0

# Filtering
alpha = 0.1  # Low pass filter
gyro_deadzone = 0.2  # Rudimentary high pass filter

# Timer
prev_time_imu = time.time()

## Navigation variables
path_distance = 0.0         # Distance travelled on preset path (use to stay within perimeter)
path_angle = 0.0            # Angle on preset path (use for maintaining heading or turning at end of preset path)
garbage_distance = 0        # Distance travelled when collecting garbage (use to travel to and from detected objects)
garbage_angle = 0.0         # Adjusted angle when collecting garbage (use to turn to and from detected objects)

# Garbage distance tracking
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

# Garbage angle tracking
garbage_angle_flag = False
initial_garbage_angle = 0.0

def start_garbage_angle():
    global garbage_angle_flag, initial_garbage_angle
    garbage_angle_flag = True
    initial_garbage_angle = angle
    
def update_garbage_angle():
    global garbage_angle, filtered_gyro, prev_time_imu
    if garbage_angle_flag:
        # IMU calculations
        current_time_imu = time.time()
        dt = current_time_imu - prev_time_imu
        prev_time_imu = current_time_imu
        
        gyro = imu_sensor.gyro
        if gyro is not None:
            raw_gyro = gyro[2] * 180 / pi
            filtered_gyro = alpha * raw_gyro + (1 - alpha) * filtered_gyro  # Low pass filter
            if abs(filtered_gyro) < gyro_deadzone: filtered_gyro = 0        # Rudimentry high pass filter
            
            # Integrate angular velocity to calculate angle
            garbage_angle += filtered_gyro * dt
            # Normalized angle difference
            garbage_angle = (garbage_angle - initial_garbage_angle) % 360
            
    return garbage_angle
        
def reset_garbage_angle():
    global garbage_angle_flag
    garbage_angle_flag = False

# Path distance tracking
path_distance_flag = False
path_distance_y = 0.0
path_distance_y_initial = 0.0
path_distance_x = 0.0
path_distance_x_initial = 0.0

def start_path_distance(direction):
    global path_distance_flag, path_distance_x_initial, path_distance_y_initial
    path_distance_flag = True
    if direction == "x":
        path_distance_x_initial = (left_count + right_count) * cm_per_pulse / 2
    elif direction == "y":
        path_distance_y_initial = (left_count + right_count) * cm_per_pulse / 2
    
def update_path_distance(direction):
    global path_distance_x, path_distance_y
    
    if direction == "x":
        if garbage_distance_flag or garbage_angle_flag:
            return path_distance_x
        if path_distance_flag:
            path_distance_x_new = (left_count + right_count) * cm_per_pulse / 2
            path_distance_x = path_distance_x_new - path_distance_x_initial
        return path_distance_x
        
    elif direction == "y":
        if garbage_distance_flag or garbage_angle_flag:
            return path_distance_y
        if path_distance_flag:
            path_distance_y_new = (left_count + right_count) * cm_per_pulse / 2
            path_distance_y = path_distance_y_new - path_distance_y_initial
        return path_distance_y

def reset_path_distance(direction):
    global path_distance_flag, path_distance_x, path_distance_y
    path_distance_flag = False
    
    if direction == "x":
        path_distance_x = 0.0
    elif direction == "y":
        path_distance_y = 0.0

# Path angle tracking
path_angle_flag = False
path_angle = 0.0
path_angle_initial = 0.0

def start_path_angle():
    global path_angle_flag, path_angle_initial
    path_angle_flag = True
    path_angle_initial = angle

def update_path_angle():
    global path_angle
    if garbage_distance_flag or garbage_angle_flag:
        return path_angle

    if path_angle_flag:
        path_angle = (angle - path_angle_initial) % 360

    return path_angle

def reset_path_angle():
    global path_angle_flag, path_angle, path_angle_initial
    path_angle_flag = False
    path_angle = 0.0