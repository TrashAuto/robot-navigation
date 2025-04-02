# Rotary wheel encoder and BNO055 IMU-powered navigation system on Raspberry Pi 5 for distance and angle tracking

# Import libraries
import time
from gpiozero import Button
from math import pi
import adafruit_bno055
import busio
import board

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

print("Outputting encoder distance and filtered yaw angle (deg):\n")

# Main loop
try:
    while True:
        time.sleep(0.1)
        # Encoder calculations
        elapsed_encoder = time.time() - start_time_encoder
        left_distance = left_count * cm_per_pulse
        right_distance = right_count * cm_per_pulse
        avg_distance = (left_distance + right_distance) / 2

        # IMU calculations
        current_time_imu = time.time()
        dt = current_time_imu - prev_time_imu
        prev_time_imu = current_time_imu
        elapsed_imu = current_time_imu - start_time_imu

        gyro = sensor.gyro
        if gyro is not None:
            raw_gyro = gyro[2] * 180 / pi

            # Low pass filter
            filtered_gyro = alpha * raw_gyro + (1 - alpha) * filtered_gyro

            # High pass filter
            if abs(filtered_gyro) < gyro_deadzone:
                filtered_gyro = 0

            # Integrate angular velocity to calculate angle
            angle += filtered_gyro * dt

            # Normalize angle from 0 to 360 degrees
            angle = angle % 360 + 360

        # Print output
        print(f"Time (Encoder): {elapsed_encoder:.1f}s, Time (IMU): {elapsed_imu:.1f}s")
        print(f"  Right count, distance: {right_count}, {right_distance:.2f} cm")
        print(f"  Left count, distance: {left_count}, {left_distance:.2f} cm")
        print(f"  Averaged distance: {avg_distance:.2f} cm")
        print(f"  Angle (deg): {angle:.2f}")
        print(f"  Raw gyro: {raw_gyro:.2f}, Filtered: {filtered_gyro:.2f}\n")

except KeyboardInterrupt:
    print("Exiting...")