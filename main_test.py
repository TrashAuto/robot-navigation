# Import libraries
import math
import time

# Import modules
from motion import move_forward_until, move_backward_until, turn_left_until, turn_right_until
from gpio import collect_garbage
from classification import run_ml_pipeline
from perception import detect_object_of_interest, is_tall_object_present
from navigation import start_path_distance, update_path_distance, reset_path_distance

# Declare parameters (all distances in cm)
PERIMETER_X = 5
PERIMETER_Y = 5
observed_distance = 50
observed_angle = math.radians(45)
side_distance = math.cos(observed_angle) * observed_distance / math.sin(observed_angle)
collection_distance = 10 # Distance between garbage and LiDAR when collecting garbage

def loop(PERIMETER_X, PERIMETER_Y):
    start_path_distance("y")
    start_path_distance("x")
    
    while True:
        # Travel 50 cm and stop
        move_forward_until(50, "path", "y")
        
        print("Travelled 50 cm, running LiDAR scan...")
        
        # Run LiDAR scan after stopping
        object = detect_object_of_interest()
        
        if object:
            print("Object detected!")
            
            object_angle = object['relative_angle_deg']
            object_distance = object['distance_mm'] / 10  # Convert to cm
            object_width = object['width_mm'] / 10
            
            if abs(object_angle) > 10 and object_distance > 20:
                print("Object is off the path. Running object_event_off_path...")
                # Object is off the path (confirm requirements)
                object_event_off_path(object_distance, object_angle)
            else:
                print("Object is on the path. Running object_event_on_path...")
                # Object is on the path
                object_event_on_path(object_distance, object_width)
        
        distance_travelled_y = update_path_distance("y")
        distance_travelled_x = update_path_distance("x")

        if distance_travelled_y >= 0.9 * PERIMETER_Y:
            print("Reached 90% of PERIMETER_Y, turning right...")
            turn_right_until(90, "path")
            move_forward_until(side_distance * 1.5, "path", "x")
            turn_right_until(90, "path")
            reset_path_distance("y")
            start_path_distance("y")

        if distance_travelled_x >= 0.9 * PERIMETER_X:
            print("Reached 90% of PERIMETER_X, turning right...")
            turn_right_until(90, "path")
            move_forward_until(0.9 * PERIMETER_X, "path", "y")
            turn_right_until(90, "path")
            reset_path_distance("x")
            start_path_distance("x")
        
        time.sleep(0.1)
            
def object_event_off_path(object_distance, object_angle):
    if object_angle < 0:
        print("Object angle is negative, turning left...")
        turn_left_until(-object_angle, "object") 
    else: 
        print("Object angle is positive, turning right...")
        turn_right_until(object_angle, "object")
    
    print("Moving forward to garbage...")
    move_forward_until(object_distance - collection_distance, "object")
    
    if not is_tall_object_present(object_distance * 1000) and run_ml_pipeline():
        print("Object not tall and classified as garbage, collecting garbage...")
        collect_garbage()
        
    print("Moving backward to original position...")
    move_backward_until(object_distance - collection_distance, "object")
    
    if object_angle < 0:
        print("Object angle is negative, turning right...")
        turn_right_until(-object_angle, "object")
    else:
        print("Object angle is positive, turning left...")
        turn_left_until(object_angle, "object")
        
def object_event_on_path(object_distance, object_width):
    print("Moving forward to garbage...")
    move_forward_until(object_distance - collection_distance, "object")
    if not is_tall_object_present(object_distance * 1000) and run_ml_pipeline():
        print("Object not tall and classified as garbage, collecting garbage...")
        collect_garbage()
    else:
        # Move around object
        print("Object is too tall or not classified as garbage, moving around object...")
        obstacle_event(object_width)
    
def obstacle_event(object_width):
    turn_right_until(90, "object")
    move_forward_until(3 * object_width)
    turn_left_until(90, "object")
    move_forward_until(3 * object_width)
    turn_left_until(90, "object")
    move_forward_until(3 * object_width)
    turn_right_until(90, "object")

if __name__ == "__main__":
    loop(PERIMETER_X, PERIMETER_Y)