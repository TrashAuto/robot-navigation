import motion

## Test IMU functions
print("IMU testing started.")

print("Turn IMU right 90 degrees.")
motion.turn_right_until(90)
print("Turned right 90 degrees...")

print("Turn IMU left 90 degrees.")
motion.turn_left_until(90)
print("Turned left 90 degrees...")

print("Turn IMU right 45 degrees.")
motion.turn_right_until(45)
print("Turned right 45 degrees...")

print("Turn IMU left 90 degrees.")
motion.turn_left_until(90)
print("Turned leftt 90 degrees...")

print("IMU testing complete.")