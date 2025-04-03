import time
import navigation
import gpio

def move_forward_until(distance_cm, mode):
    if mode == "path":
        navigation.start_path_distance()
    elif mode == "object":
        navigation.start_garbage_distance()
    
    gpio.move_forward()
    
    while True:
        if mode == "path":
            distance = navigation.update_path_distance()
        elif mode == "object":
            distance = navigation.update_garbage_distance()
            
        if distance >= distance_cm:
            gpio.stop_moving()
            break
        
        time.sleep(0.1)

def move_backward_until(distance_cm, mode):
    if mode == "path":
        navigation.start_path_distance()
    elif mode == "object":
        navigation.start_garbage_distance()
    
    gpio.move_backward()
    
    while True:
        if mode == "path":
            distance = navigation.update_path_distance()
        elif mode == "object":
            distance = navigation.update_garbage_distance()
            
        if distance >= distance_cm:
            gpio.stop_moving()
            break
        
        time.sleep(0.1)

def turn_right_until(angle_deg, mode):
    if mode == "path":
        navigation.start_path_angle()
    elif mode == "object":
        navigation.start_garbage_angle()
    
    gpio.turn_right()
    
    while True:
        if mode == "path":
            angle = navigation.update_path_angle()
        elif mode == "object":
            angle = navigation.update_garbage_angle()
            
        if angle >= angle_deg:
            gpio.stop_turning()
            break
        
        time.sleep(0.1)

def turn_left_until(angle_deg, mode):
    if mode == "path":
        navigation.start_path_angle()
    elif mode == "object":
        navigation.start_garbage_angle()
    
    gpio.turn_left()
    
    while True:
        if mode == "path":
            angle = navigation.update_path_angle()
        elif mode == "object":
            angle = navigation.update_garbage_angle()
            
        if angle >= angle_deg:
            gpio.stop_turning()
            break
        
        time.sleep(0.1)