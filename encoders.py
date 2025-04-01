from gpiozero import Button
import time
import math

# GPIO Pins
left_A_pin = 4
left_B_pin = 17
right_A_pin = 27
right_B_pin = 22

# Encoder Parameters
pulses_per_rev = 12
decoding_factor = 2
effective_counts_per_rev = pulses_per_rev * decoding_factor
wheel_radius_cm = 10
cm_per_pulse = 2 * math.pi * wheel_radius_cm / effective_counts_per_rev

# State
left_count = 0
right_count = 0
start_time = time.time()

# Noise rejection
min_pulse_interval = 0.001  # 1 ms between pulses (1000 Hz max)

# Last pulse timestamps
last_left_time = 0
last_right_time = 0

# Channel B inputs
left_B = Button(left_B_pin, pull_up=True)
right_B = Button(right_B_pin, pull_up=True)

# Channel A interrupt definitions
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

# Channel A interrupts (on both edges for 2x decoding)
left_A = Button(left_A_pin, pull_up=True)
left_A.when_pressed = on_left_A
left_A.when_released = on_left_A

right_A = Button(right_A_pin, pull_up=True)
right_A.when_pressed = on_right_A
right_A.when_released = on_right_A

# Loop
try:
    while True:
        time.sleep(1)
        elapsed = time.time() - start_time

        left_distance = left_count * cm_per_pulse
        right_distance = right_count * cm_per_pulse

        print(f"Time: {elapsed:.1f}s")
        print(f"  Left  | Pulses: {left_count} | Distance: {left_distance:.2f} cm")
        print(f"  Right | Pulses: {right_count} | Distance: {right_distance:.2f} cm\n")

except KeyboardInterrupt:
    print("Exiting cleanly...")