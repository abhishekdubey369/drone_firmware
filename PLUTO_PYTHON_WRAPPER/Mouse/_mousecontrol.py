from pynput import mouse
from plutocontrol import pluto
from threading import Thread
import time

# Initialize and connect to the drone
my_pluto = pluto()
my_pluto.connect()
my_pluto.arm()
time.sleep(2)

#mouse initialization
# mouse = controller()

# Global flag to control the listener loop
keep_running = True

def identify_mouse_action(action, x, y, button, pressed, dx, dy,keep_run):
    if action == 'click':
        if button == mouse.Button.left and pressed:
            if keep_running:
                print("Left click: Taking off")
                my_pluto.take_off()
        elif button == mouse.Button.right and pressed:
            if keep_running:
                print("Right click: Landing")
                my_pluto.land()
    elif keep_running and action == 'move':
        print(f"Mouse moved to ({x}, {y})")
        if x < 100:
            print("Moving left")
            my_pluto.left()
        elif x > 500:
            print("Moving right")
            my_pluto.right()
        if y < 100:
            print("Moving forward")
            my_pluto.forward()
        elif y > 500:
            print("Moving backward")
            my_pluto.backward()
    elif keep_running and action == 'scroll':
        print(f"Mouse scrolled at ({x}, {y}) with delta ({dx}, {dy})")
        if dy > 0:
            print("Increasing height")
            my_pluto.increase_height()
        elif dy < 0:
            print("Decreasing height")
            my_pluto.decrease_height()
    elif keep_running!=True:
        clean_exit()

# Event handlers for mouse
def on_click(x, y, button, pressed):
    identify_mouse_action('click', x, y, button, pressed, None, None,keep_running)

def on_move(x, y):
    identify_mouse_action('move', x, y, None, None, None, None,keep_running)

def on_scroll(x, y, dx, dy):
    identify_mouse_action('scroll', x, y, None, None, dx, dy,keep_running)

def start_listener():
    # Start the mouse listener in a separate thread
    print("Starting mouse listener...")
    if(keep_running!=True):
        listener.join()
    else:
        listener =  mouse.Listener(on_click=on_click, on_move=on_move, on_scroll=on_scroll)
        listener.start()
    

def clean_exit():
    global keep_running
    keep_running = False
    try:
        print("Disarming...")
        my_pluto.disarm()
        time.sleep(2)
        print("Disconnecting...")
        my_pluto.disconnect()
    except Exception as e:
        print(f"Exception during cleanup: {e}")
    finally:
        print("Exiting...")
        exit()

# Main loop
if __name__ == "__main__":
    listener_thread = Thread(target=start_listener)
    listener_thread.start()

    try:
        while keep_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        keep_running = False
        print("KeyboardInterrupt caught. Exiting...")
        clean_exit()
    
    listener_thread.join()

    # Ensure listener thread exits properly
    
