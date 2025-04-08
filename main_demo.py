# Import libraries
import math
import time
import threading

# Import modules
from motion import move_forward_until, move_backward_until, turn_left_until, turn_right_until
from gpio import collect_garbage
from classification import run_ml_pipeline
from perception import detect_object_of_interest, is_tall_object_present
from navigation import (start_path_distance, update_path_distance, reset_path_distance, start_deviation_angle,
                        update_deviation_angle)

# Declare parameters (all distances in cm)
PERIMETER_X = 10
PERIMETER_Y = 10
observed_distance = 5
observed_angle = math.radians(45)
side_distance = math.cos(observed_angle) * observed_distance / math.sin(observed_angle)
collection_distance = 0 # Distance between garbage and LiDAR when collecting garbage
facing_up = True

def loop(PERIMETER_X, PERIMETER_Y):
    print("Running with a perimeter of 10x10 cm.")
    global facing_up
    
    # Initialize path distances
    start_path_distance("y")
    start_path_distance("x")
    
    while True:
        # Travel 50 cm and stop
        print("Moving forward 5 cm")
        move_forward_until(5, "path", "y")
        print("Moved forward 5 cm")
        
        # Run LiDAR scan after stopping
        print("Scanning for objects")
        object = detect_object_of_interest()
        
        if object:
            print("Object detected")
            object_angle = object['relative_angle_deg']
            object_distance = object['distance_mm'] / 10  # Convert to cm
            object_width = object['width_mm'] / 10
            
            if abs(object_angle) > 10 and object_distance > 20:
                print("Object is off the path")
                # Object is off the path (confirm requirements)
                object_event_off_path(object_distance, object_angle)
            else:
                print("Object is on the path")
                # Object is on the path
                object_event_on_path(object_distance, object_width)
        
        distance_travelled_y = update_path_distance("y")
        distance_travelled_x = update_path_distance("x")
        
        if distance_travelled_y >= 0.9 * PERIMETER_Y:
            print("Reached the end of the Y path")
            if facing_up:
                turn_right_until(90)
                move_forward_until(side_distance * 1.5, "path", "x")
                turn_right_until(90)
            else:
                turn_left_until(90)
                move_forward_until(side_distance * 1.5, "path", "x")
                turn_left_until(90)
            
            facing_up = not facing_up
            reset_path_distance("y")
            start_path_distance("y")

        if distance_travelled_x >= 0.9 * PERIMETER_X:
            print("Reached the end of the X path")
            move_backward_until(update_path_distance("x"), "path", "x")
            if facing_up:
                turn_left_until(90)
                move_backward_until(update_path_distance("y"), "path", "y")
            else:
                turn_left_until(90)
                facing_up = not facing_up
                
            reset_path_distance("x")
            start_path_distance("x")
            reset_path_distance("y")
            start_path_distance("y")
    
        time.sleep(0.1)
            
def object_event_off_path(object_distance, object_angle):
    if object_angle < 0: 
        print("Turning left to align with object")
        turn_left_until(-object_angle) 
        print("Done turning left")
    else:
        print("Turning right to align with object")
        turn_right_until(object_angle)
        print("Done turning right")
    
    print("Moving forward to collect garbage")
    move_forward_until(object_distance - collection_distance, "object")
    print("Done moving forward, checking if object is garbage")
    
    if not is_tall_object_present(object_distance * 10) and run_ml_pipeline():
        print("Collecting garbage")
        collect_garbage()
        
    print("Moving backward to original position")
    move_backward_until(object_distance - collection_distance, "object")
    print("Done moving backward, returning to original orientation")
    
    if object_angle < 0:
        print("Turning right to return to original orientation")
        turn_right_until(-object_angle)
    else:
        print("Turning left to return to original orientation")
        turn_left_until(object_angle)
        
def object_event_on_path(object_distance, object_width):
    print("Moving forward to collect garbage")
    move_forward_until(object_distance - collection_distance, "path", "y")
    if not is_tall_object_present(object_distance * 10) and run_ml_pipeline():
        print("Collecting garbage")
        collect_garbage()
    else:
        # Move around object
        print("Object is not garbage, moving around it")
        obstacle_event(object_width)
        print("Done moving around object")
    
def obstacle_event(object_width):
    print("Turning right 90 degrees")
    turn_right_until(90)
    print("DOne turning right, moving forward to bypass object")
    move_forward_until(3 * object_width, "path", "x")
    print("Done moving forward, turning left 90 degrees to bypass object")
    turn_left_until(90)
    print("Done turning left, moving forward to bypass object")
    move_forward_until(3 * object_width, "path", "y")
    print("Done moving forward, turning right 90 degrees")
    turn_right_until(90)
    print("Done turning right, moving backward to original path")
    move_backward_until(3 * object_width, "path", "x")
    print("Done moving backward, turning left 90 degrees to return to original orientation")
    turn_left_until(90)
    print("Done turning left. Obstacle avoided!")

def deviation_angle_correction():
    while True:
        deviation = update_deviation_angle()
        if deviation > 10:
            print("Correcting deviation to the left")
            turn_left_until(deviation)
        elif deviation < -10:
            print("Correcting deviation to the right")
            turn_right_until(-deviation)
        time.sleep(0.1)

if __name__ == "__main__":
    start_deviation_angle()
    thread_deviation_correction = threading.Thread(target=deviation_angle_correction, daemon=True)
    thread_deviation_correction.start()
    loop(PERIMETER_X, PERIMETER_Y)