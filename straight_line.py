import time
from motion import move_forward_until, move_backward_until
from gpio import collect_garbage
from classification import run_ml_pipeline
from perception import is_tall_object_present

def demo_loop():
    while True:
        print("Moving forward...")
        move_forward_until(30, mode="path", direction="y")  # move 30cm forward
        print("Stopped. Running garbage detection...")

        if not is_tall_object_present:
            if run_ml_pipeline():
                print("Garbage detected! Activating collection mechanism...")
                collect_garbage()
            else: 
                print("No garbage detected.")
        else:
            print("No garbage detected.")

        print("Reversing...")
        move_backward_until(30, mode="path", direction="y")  # move 30cm backward

        print("Cycle complete. Restarting...\n")
        time.sleep(1)

if __name__ == "__main__":
    demo_loop()