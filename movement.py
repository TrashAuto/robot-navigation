import gpiozero as GPIO
import time

# Declare GPIO pins as outputs
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(25, GPIO.OUT)

# Default GPIO output: 000
GPIO.output(23, GPIO.LOW)
GPIO.output(24, GPIO.LOW)
GPIO.output(25, GPIO.LOW)

# Motor control function definitions
def move_forward(): # Move forward: 001
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(25, GPIO.HIGH)

def move_backward(): # Move backward: 010
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.LOW)
    
def stop_moving(): # Stop moving: 011
    GPIO.output(23, GPIO.LOW)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.HIGH)

def turn_right(): # Turn right: 100
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(25, GPIO.LOW)

def turn_left(): # Turn left: 101
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.LOW)
    GPIO.output(25, GPIO.HIGH)

def stop_turning(): # Stop turning: 110
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.LOW)

def collect_garbage(): # Collect garbage: 111
    GPIO.output(23, GPIO.HIGH)
    GPIO.output(24, GPIO.HIGH)
    GPIO.output(25, GPIO.HIGH)