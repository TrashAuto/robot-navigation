import time
import adafruit_bno055
import busio
import board

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Initialize state variables
velocity = [0.0, 0.0, 0.0]   # x, y, z velocity
position = [0.0, 0.0, 0.0]   # x, y, z position
angle = [0.0, 0.0, 0.0]      # x, y, z angle (degrees)

prev_time = time.time()

# Integrate acceleration for distance
def integrate_accel(acc, vel, pos, dt):
    vel_new = [vel[i] + acc[i] * dt for i in range(3)]
    pos_new = [pos[i] + vel_new[i] * dt for i in range(3)]
    return vel_new, pos_new

# Integrate angular velocity for angle
def integrate_gyro(gyro, angle, dt):
    return [angle[i] + (gyro[i] * dt * 180 / 3.14159) for i in range(3)]

print("Outputting distance (m) and angle (deg):\n")

while True:
    curr_time = time.time()
    dt = curr_time - prev_time
    prev_time = curr_time

    acc = sensor.linear_acceleration
    gyro = sensor.gyro

    if acc is not None and gyro is not None:
        velocity, position = integrate_accel(acc, velocity, position, dt)
        angle = integrate_gyro(gyro, angle, dt)

        print(f"Distance (m): {[round(p, 2) for p in position]}")
        print(f"Angle (deg): {[round(a, 2) for a in angle]}")
        print()

    time.sleep(0.1)