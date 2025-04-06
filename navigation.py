# Import libraries
import time
from math import pi
from gpiozero import Button
from smbus2 import SMBus
import motion

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
r = 3.65  # Radius
cm_per_pulse = 2 * pi * r / eff_cpr

# State
left_count = 0  # Directional net count
right_count = 0
left_distance = 0.0  # Total distance regardless of direction
right_distance = 0.0
start_time_encoder = time.time()

# Motion direction
current_direction = 1
def set_direction(direction):
    global current_direction
    current_direction = direction

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
        left_count += current_direction
        last_left_time = now

def on_right_A():
    global right_count, last_right_time
    now = time.time()
    if now - last_right_time >= min_pulse_interval:
        right_count += current_direction
        last_right_time = now

# Set up channel A with rising (released) and falling (pressed) edges
left_button = Button(left_A, pull_up=False)
right_button = Button(right_A, pull_up=False)
left_button.when_pressed = on_left_A
left_button.when_released = on_left_A
right_button.when_pressed = on_right_A
right_button.when_released = on_right_A

## BNO055 IMU setup ##

BNO055_ADDRESS = 0x28
BNO055_CHIP_ID_ADDR = 0x00
BNO055_OPR_MODE_ADDR = 0x3D
BNO055_PWR_MODE_ADDR = 0x3E
BNO055_UNIT_SEL_ADDR = 0x3B

CONFIGMODE = 0X00
NDOF_MODE = 0X0C

GYRO_DATA_ADDR = 0x14

bus = SMBus(1)

# Read gyroscope data
def read_gyro_z():
    data = bus.read_i2c_block_data(BNO055_ADDRESS, GYRO_DATA_ADDR, 6)
    z = int.from_bytes(data[4:6], byteorder='little', signed=True)
    return z / 16.0  # degrees/s

# State (z axis only for 2D angle tracking)
filtered_gyro = 0.0  # Filtered angular velocity

# Filtering
alpha = 0.1  # Low pass filter
gyro_deadzone = 0.2  # Rudimentary high pass filter

## Navigation variables
path_distance_x = 0.0       # Distance travelled on preset path (use to stay within perimeter)
path_distance_y = 0.0       
garbage_distance = 0        # Distance travelled when collecting garbage (use to travel to and from detected objects)
angle = 0.0                 # Adjusted angle when turning

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
        if garbage_distance_flag:
            return path_distance_x
        if path_distance_flag:
            path_distance_x_new = (left_count + right_count) * cm_per_pulse / 2
            path_distance_x = path_distance_x_new - path_distance_x_initial
        return path_distance_x
        
    elif direction == "y":
        if garbage_distance_flag:
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

# Turn angle tracking for perimeter turns and garbage collection
angle_flag = False
prev_angle_time = 0.0
angle = 0.0

def start_angle():
    global angle_flag, prev_angle_time
    angle_flag = True
    prev_angle_time = time.time()
    
def update_angle():
    global angle, filtered_gyro, prev_angle_time
            
    if angle_flag:
        # IMU calculations
        current_time_imu = time.time()
        dt = current_time_imu - prev_angle_time
        prev_angle_time = current_time_imu
         
        raw_gyro = read_gyro_z()

        # Low pass filter
        filtered_gyro = alpha * raw_gyro + (1 - alpha) * filtered_gyro
        # Rudimentary high pass filter
        if abs(filtered_gyro) < gyro_deadzone:                          
            filtered_gyro = 0

        # Integrate angular velocity to calculate angle
        angle += filtered_gyro * dt
        # Normalize angle from 0 to 180 degrees
        angle = (angle + 180) % 360 - 180
            
    return angle
        
def reset_angle():
    global angle_flag, angle
    angle_flag = False
    angle = 0.0

# Path angle tracking for deviation correction
prev_deviation_angle_time = 0.0
deviation_angle = 0.0

def start_deviation_angle():
    global prev_deviation_angle_time
    prev_deviation_angle_time = time.time()

def update_deviation_angle():
    global deviation_angle, filtered_gyro, prev_deviation_angle_time
    
    if not angle_flag:
        # IMU calculations
        current_time_imu = time.time()
        dt = current_time_imu - prev_deviation_angle_time
        prev_deviation_angle_time = current_time_imu
         
        raw_gyro = read_gyro_z()

        # Low pass filter
        filtered_gyro = alpha * raw_gyro + (1 - alpha) * filtered_gyro
        # Rudimentary high pass filter
        if abs(filtered_gyro) < gyro_deadzone:                          
            filtered_gyro = 0

        # Integrate angular velocity to calculate deviation angle
        deviation_angle += filtered_gyro * dt
        # Normalize deviation angle from 0 to 180 degrees
        deviation_angle = (deviation_angle + 180) % 360 - 180
            
    return angle

def deviation_angle_correction():
    while True:
        deviation = update_deviation_angle()
        if deviation > 10:
            motion.turn_left_until(deviation)
        elif deviation < -10:
            motion.turn_right_until(-deviation)
        time.sleep(0.1)