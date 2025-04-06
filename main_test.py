# Import libraries
import math
import time
import threading

# Import modules
from motion import move_forward_until, move_backward_until, turn_left_until, turn_right_until
from gpio import collect_garbage
from classification import run_ml_pipeline
from perception import detect_object_of_interest, is_tall_object_present
from navigation import (
    start_path_distance, update_path_distance, reset_path_distance,
    start_deviation_angle, deviation_angle_correction
)

# Declare parameters (all distances in cm)
PERIMETER_X = 20
PERIMETER_Y = 20
observed_distance = 5
observed_angle = math.radians(45)
side_distance = math.cos(observed_angle) * observed_distance / math.sin(observed_angle)
collection_distance = 10  # Distance between garbage and LiDAR when collecting garbage
facing_up = True

def loop(PERIMETER_X, PERIMETER_Y, facing_up):
    start_path_distance("y")
    start_path_distance("x")
    print("Starting path distances (x and y)...")

    while True:
        print("Moving forward 4.5 cm on Y axis...")
        move_forward_until(4.5, "path", "y")

        print("Running LiDAR scan...")
        object = detect_object_of_interest()

        if object:
            print("Object detected!")
            object_angle = object['relative_angle_deg']
            object_distance = object['distance_mm'] / 10  # Convert to cm
            object_width = object['width_mm'] / 10

            if abs(object_angle) > 10 and object_distance > 20:
                print("Object is off path. Running off-path event handler...")
                object_event_off_path(object_distance, object_angle)
            else:
                print("Object is on path. Running on-path event handler...")
                object_event_on_path(object_distance, object_width)

        distance_travelled_y = update_path_distance("y")
        distance_travelled_x = update_path_distance("x")

        print(f"Distance Y: {distance_travelled_y:.2f} cm, Distance X: {distance_travelled_x:.2f} cm")

        if distance_travelled_y >= 0.9 * PERIMETER_Y:
            print("Reached end of Y path. Turning...")
            if facing_up:
                print("Turning right 90 degrees..")
                turn_right_until(90)
                print("Turned right 90 degrees, moving forward...")
                move_forward_until(side_distance * 1.5, "path", "x")
                print("Moved forward, turning right 90 degrees..")
                turn_right_until(90)
                print("Turned right 90 degrees.")
            else:
                print("Turning left 90 degrees..")
                turn_left_until(90)
                print("Turned left 90 degrees, moving forward...")
                move_forward_until(side_distance * 1.5, "path", "x")
                print("Moved forward, turning left 90 degrees...")
                turn_left_until(90)
                print("Turned left 90 degrees.")

            facing_up = not facing_up
            reset_path_distance("y")
            start_path_distance("y")
            print("Turn complete. Resuming path in new direction.")

        if distance_travelled_x >= 0.9 * PERIMETER_X:
            print("Reached end of X path. Resetting path...")
            print("Moving backward until path distance is reset...")
            move_backward_until(update_path_distance("x"), "path", "x")
            if facing_up:
                print("Turning left 90 degrees...")
                turn_left_until(90)
                print("Turned left 90 degrees, moving backward...")
                move_backward_until(update_path_distance("y"), "path", "y")
            else:
                print("Turning left 90 degrees...")
                turn_left_until(90)
                print("Turned left 90 degrees")
                facing_up = not facing_up

            print("Resetting path distances...")
            reset_path_distance("x")
            start_path_distance("x")
            reset_path_distance("y")
            start_path_distance("y")
            print("Finished resetting path.")

        time.sleep(0.1)

def object_event_off_path(object_distance, object_angle):
    if object_angle < 0:
        print(f"Object angle is negative, turning left by {-object_angle:.2f} degrees...")
        turn_left_until(-object_angle)
    else:
        print(f"Object angle is negative, turning right by {object_angle:.2f} degrees...")
        turn_right_until(object_angle)

    print("Moving toward object...")
    move_forward_until(object_distance - collection_distance)

    if not is_tall_object_present(object_distance * 10) and run_ml_pipeline():
        print("Garbage detected. Collecting...")
        collect_garbage()

    print("Moving backward to return to original path...")
    move_backward_until(object_distance - collection_distance)

    if object_angle < 0:
        print(f"Object angle is negative, turning right by {-object_angle:.2f} degrees...")
        turn_right_until(-object_angle)
    else:
        print(f"Object angle is positive, turning left by {object_angle:.2f} degrees...")
        turn_left_until(object_angle)

def object_event_on_path(object_distance, object_width):
    print("Approaching object on path...")
    move_forward_until(object_distance - collection_distance)
    if not is_tall_object_present(object_distance * 10) and run_ml_pipeline():
        print("Garbage detected. Collecting...")
        collect_garbage()
    else:
        print("Object not garbage or is tall. Avoiding obstacle...")
        obstacle_event(object_width)

def obstacle_event(object_width):
    print("Avoiding obstacle on path, turning right 90 degrees...")
    turn_right_until(90)
    print("Turned right 90 degrees, moving forward until 3 times the object width...")
    move_forward_until(3 * object_width, "path", "x")
    print("Moved forward, turning left 90 degrees...")
    turn_left_until(90)
    print("Turned left 90 degrees, moving forward until 3 times the object width...")
    move_forward_until(3 * object_width, "path", "y")
    print("Moved forward, turning left 90 degrees...")
    turn_left_until(90)
    print("Turned left 90 degrees, moving forward until 3 times the object width...")
    move_forward_until(3 * object_width, "path", "x")
    print("Moved forward, turning right 90 degrees...")
    turn_right_until(90)
    print("Turned right 90 degrees, obstacle avoided. Back on path.")

if __name__ == "__main__":
    print("Starting deviation correction thread...")
    start_deviation_angle()
    thread_deviation_correction = threading.Thread(target=deviation_angle_correction, daemon=True)
    thread_deviation_correction.start()
    print("Starting main loop...")
    loop(PERIMETER_X, PERIMETER_Y, facing_up)