import time
import math
from gpiozero import Button
import board
import busio
import adafruit_bno055

# Encoder GPIO Pins
left_A_pin = 4
left_B_pin = 17
right_A_pin = 27
right_B_pin = 22

# Encoder parameters
pulses_per_rev = 12
decoding_factor = 2
effective_counts_per_rev = pulses_per_rev * decoding_factor
wheel_radius_cm = 10
cm_per_pulse = 2 * math.pi * wheel_radius_cm / effective_counts_per_rev

# Encoder state
left_count = 0
right_count = 0
last_left_time = 0
last_right_time = 0
min_pulse_interval = 0.001  # 1 ms for noise rejection

# B channels
left_B = Button(left_B_pin, pull_up=True)
right_B = Button(right_B_pin, pull_up=True)

# Channel A interrupts for 2x decoding
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

# Channel A setup with double edge detection
left_A = Button(left_A_pin, pull_up=True)
left_A.when_pressed = on_left_A
left_A.when_released = on_left_A

right_A = Button(right_A_pin, pull_up=True)
right_A.when_pressed = on_right_A
right_A.when_released = on_right_A

# IMU setup
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

velocity = [0.0, 0.0, 0.0]   # m/s
position = [0.0, 0.0, 0.0]   # meters
prev_time = time.time()

# Fuse IMU and encoders
encoder_weight = 0.8
imu_weight = 0.2

prev_encoder_dist = 0.0
prev_imu_dist = 0.0
fused_distance = 0.0

# Loop
print("Fusing encoder + IMU distance...\n")
try:
    while True:
        time.sleep(0.1)

        # Time delta
        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        # Encoder distance
        left_dist_cm = left_count * cm_per_pulse
        right_dist_cm = right_count * cm_per_pulse
        encoder_dist_cm = (left_dist_cm + right_dist_cm) / 2
        encoder_dist_m = encoder_dist_cm / 100

        encoder_delta = encoder_dist_m - prev_encoder_dist
        prev_encoder_dist = encoder_dist_m

        # Integrate IMU acceleration for distance
        acc = sensor.linear_acceleration  # m/s^2
        if acc is not None:
            # Integrate acceleration -> velocity -> position
            velocity = [velocity[i] + acc[i] * dt for i in range(3)]
            position = [position[i] + velocity[i] * dt for i in range(3)]
            imu_dist_m = math.sqrt(position[0]**2 + position[1]**2 + position[2]**2)

            imu_delta = imu_dist_m - prev_imu_dist
            prev_imu_dist = imu_dist_m

            # Fused distance
            if abs(encoder_delta) > 0.0001 and abs(imu_delta) > 0.0001:
                fused_distance += encoder_weight * encoder_delta + imu_weight * imu_delta
            else:
                # Movement mismatch — likely drift or slip, discard
                print("⚠️ Drift/slip detected — not updating fused distance")

            # Output
            print(f"Encoder Dist: {encoder_dist_m:.3f} m")
            print(f"IMU Dist:     {imu_dist_m:.3f} m")
            print(f"Fused Dist:   {fused_distance:.3f} m\n")

except KeyboardInterrupt:
    print("Exiting cleanly...")