import time
import gpio

## Test GPIO functions
print("GPIO testing started")

print("Stop moving: 000")
gpio.stop_moving()
time.sleep(10)

print("Stop turning: 001")
gpio.stop_turning()
time.sleep(10)

print("Move forward: 010")
gpio.move_forward()
time.sleep(10)

print("Move backward: 011")
gpio.move_backward()
time.sleep(10)

print("Turn left: 100")
gpio.turn_left()
time.sleep(10)

print("Turn right: 101")
gpio.turn_right()
time.sleep(10)

print("Collect garbage: 110")
gpio.collect_garbage()
time.sleep(10)

print("GPIO testing complete.")