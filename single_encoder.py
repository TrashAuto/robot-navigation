from gpiozero import Button
import time
import math

# GPIO Pins for Left Encoder
left_A_pin = 4
left_B_pin = 17

# Encoder Parameters
pulses_per_rev = 12
decoding_factor = 2
effective_counts_per_rev = pulses_per_rev * decoding_factor
wheel_radius_cm = 10
cm_per_pulse = 2 * math.pi * wheel_radius_cm / effective_counts_per_rev

# State
left_count = 0
start_time = time.time()

# Noise rejection
min_pulse_interval = 0.001  # 1 ms between pulses (1000 Hz max)
last_left_time = 0

# Channel B input
left_B = Button(left_B_pin, pull_up=True)

# Channel A interrupt (2x decoding: rising + falling edge)
def on_left_A():
    global left_count, last_left_time
    now = time.time()
    if now - last_left_time >= min_pulse_interval:
        direction = -1 if left_B.is_pressed else 1
        left_count += direction
        last_left_time = now

# Setup A channel with both edge triggers
left_A = Button(left_A_pin, pull_up=True)
left_A.when_pressed = on_left_A
left_A.when_released = on_left_A

# Main loop
try:
    while True:
        time.sleep(1)
        elapsed = time.time() - start_time
        left_distance = left_count * cm_per_pulse

        print(f"Time: {elapsed:.1f}s")
        print(f"  Left  | Pulses: {left_count} | Distance: {left_distance:.2f} cm\n")

except KeyboardInterrupt:
    print("Exiting cleanly...")