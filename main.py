# Import libraries
import math
import time

# Import modules
from motion import move_forward_until, move_backward_until, turn_left_until, turn_right_until
from gpio import collect_garbage
from classification import run_ml_pipeline
from perception import detect_object_of_interest, is_tall_object_present
from navigation import start_path_distance, update_path_distance, reset_path_distance

# Declare parameters
PERIMETER_X = 10
PERIMETER_Y = 10
observed_distance = 5.0  # Example observed distance in meters
observed_angle = math.radians(45)
side_distance = math.cos(observed_angle) * observed_distance / math.sin(observed_angle)

def loop(PERIMETER_X, PERIMETER_Y):
    start_path_distance("y")
    start_path_distance("x")
    
    while True:
        move_forward_until(0.1, "path", "y")
        
        distance_travelled_y = update_path_distance("y")
        distance_travelled_x = update_path_distance("x")

        if distance_travelled_y >= 0.8 * PERIMETER_Y:
            turn_right_until(90, "path")
            move_forward_until(side_distance * 1.5, "path", "x")
            turn_right_until(90, "path")
            reset_path_distance("y")
            start_path_distance("y")

        if distance_travelled_x >= 0.8 * PERIMETER_X:
            turn_right_until(90, "path")
            move_forward_until(0.8 * PERIMETER_X, "path", "y")
            turn_right_until(90, "path")
            reset_path_distance("x")
            start_path_distance("y")
            
        if detect_object_of_interest:
            object_angle = detect_object_of_interest[0]['relative_angle_deg']
            object_distance = detect_object_of_interest[0]['distance_mm'] / 10  # Convert to cm
            if object_angle > 10 and object_distance > 20:
                # Object is off the path (confirm requirements)
                object_event_off_path(object_angle, object_distance)
            else:
                # Object is on the path
                object_event_on_path(object_angle, object_distance)
        
        time.sleep(0.1)
            
def object_event_off_path(object_angle, object_distance):
    if object_angle < 0: 
        turn_left_until(-object_angle, "object") 
    else: 
        turn_right_until(object_angle, "object")
    
    move_forward_until(object_distance, "object")
    
    if not is_tall_object_present(object_distance * 1000) and run_ml_pipeline():
        collect_garbage()
        
    move_backward_until(object_distance, "object")
    
    if object_angle < 0:
        turn_right_until(-object_angle, "object")
    else:
        turn_left_until(object_angle, "object")
        
def object_event_on_path(object_angle, object_distance):
    move_forward_until(object_distance, "object")
    if not is_tall_object_present(object_distance * 1000) and run_ml_pipeline():
        collect_garbage()
    else:
        # Move around object
        obstacle_event(object_width)
    
def obstacle_event(object_width):
    turn_right_until(90, "object")
    move_forward_until(2 * object_width)
    turn_left_until(90, "object")
    move_forward_until(2 * object_width)
    turn_left_until(90, "object")
    move_forward_until(2 * object_width)
    turn_right_until(90, "object")

if __name__ == "__main__":
    loop(PERIMETER_X, PERIMETER_Y)