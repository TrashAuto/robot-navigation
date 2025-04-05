# Import GPIO library
from gpiozero import OutputDevice

# Declare GPIO pins as outputs
pin23 = OutputDevice(23)
pin24 = OutputDevice(24)
pin25 = OutputDevice(25)

# Default GPIO output: 000
pin23.off()
pin24.off()
pin25.off()

# Motor control functions
def stop_moving():  
    # Stop moving: 000
    pin23.off()
    pin24.off()
    pin25.off()

def stop_turning():  
    # Stop turning: 001
    pin23.off()
    pin24.off()
    pin25.on()

def move_forward():  
    # Move forward: 010
    pin23.off()
    pin24.on()
    pin25.off()

def move_backward():  
    # Move backward: 011
    pin23.off()
    pin24.on()
    pin25.on()

def turn_left():  
    # Turn left: 100
    pin23.on()
    pin24.off()
    pin25.off()

def turn_right():  
    # Turn right: 101
    pin23.on()
    pin24.off()
    pin25.on()

def collect_garbage():  
    # Collect garbage: 110
    pin23.on()
    pin24.on()
    pin25.off()