# Import libraries
import math
import time

# Import modules
from sensors import get_distance, get_yaw, detect_garbage, detect_obstacle
from movement import move_forward, move_backward, stop_moving, turn_left, turn_right, stop_turning, collect_garbage

def loop(perimeter_x, perimeter_y):
    # Constants
    observed_distance = 5.0  # meters
    observed_angle = math.radians(45)
    side_distance = math.cos(observed_angle) * observed_distance / math.sin(observed_angle)

    while True:
        distance = get_distance()
        current_angle = get_yaw()

        # Decompose distance into components based on heading
        dx = distance * math.cos(math.radians(current_angle))
        dy = distance * math.sin(math.radians(current_angle))

        global distance_travelled_x, distance_travelled_y
        distance_travelled_x += dx
        distance_travelled_y += dy

        move(0.5)

        if distance_travelled_y >= 0.8 * perimeter_y:
            turn(90)
            move(side_distance * 2)
            turn(90)
            distance_travelled_y = 0.0

        if distance_travelled_x >= 0.8 * perimeter_x:
            turn(90)
            move(perimeter_x)
            turn(90)
            distance_travelled_x = 0.0

        if detect_garbage():
            garbage_event()

        if detect_obstacle():
            obstacle_event()

def garbage_event():
    turn(45)       # Replace with actual calculated garbage angle
    move(2.0)      # Replace with actual distance to garbage
    collection_mechanism()
    move(-2.0)
    turn(-45)

def obstacle_event():
    turn(90)
    move(3.0)
    turn(-90)
    move(3.0)
    turn(-90)
    move(3.0)
    turn(90)

if __name__ == "__main__":
    loop(perimeter_x=20, perimeter_y=20)