# BNO055 IMU setup on Raspberry Pi 5 for angle tracking

# Import libraries
import time
import adafruit_bno055
import busio
import board
from math import pi

# I2C setup
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# State (z axis only for 2D angle tracking)
filtered_gyro = 0.0   # Filtered angular velocity
angle = 0.0

# Filtering
alpha = 0.1             # Low pass filter
gyro_deadzone = 0.2     # High pass filter

# Timer
prev_time = time.time()
start_time = prev_time

print("Outputting filtered yaw angle (deg):\n")

try:
    while True:
        time.sleep(0.1)
        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        elapsed = current_time - start_time

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
 
            print(f"Time: {elapsed:.1f}s")
            print(f"  Angle (deg): {angle:.2f}")
            print(f"  Raw gyro: {raw_gyro:.2f}, Filtered: {filtered_gyro:.2f}\n")

except KeyboardInterrupt:
    print("Exiting...")