import cv2
import threading
import keyboard as kb
from plutocontrol import pluto

my_pluto = pluto()
webcam_thread = None
streaming = False

actions = {
    70: lambda: my_pluto.disarm() if my_pluto.rcAUX4 == 1500 else my_pluto.arm(),
    10: my_pluto.forward,
    30: my_pluto.left,
    40: my_pluto.right,
    80: my_pluto.reset,
    50: my_pluto.increase_height,
    60: my_pluto.decrease_height,
    110: my_pluto.backward,
    130: my_pluto.take_off,
    140: my_pluto.land,
    150: my_pluto.left_yaw,
    160: my_pluto.right_yaw,
    120: lambda: (print("Developer Mode ON"), setattr(my_pluto, 'rcAUX2', 1500)),
    200: my_pluto.connect,
    210: my_pluto.disconnect,
}

keyboard_cmds = {  # dictionary containing the key pressed and value associated with it
    '[A': 10, '[D': 30, '[C': 40, 'w': 50, 's': 60, ' ': 70, 'r': 80, 't': 90,
    'p': 100, '[B': 110, 'n': 120, 'q': 130, 'e': 140, 'a': 150, 'd': 160,
    '+': 15, '1': 25, '2': 30, '3': 35, '4': 45, 'c': 200, 'x': 210,
    'v': 220  # Add key 'v' to start/stop the webcam stream
}

def getKey():
    event = kb.read_event()
    if event.event_type == kb.KEY_DOWN:
        key_map = {'up': '[A', 'down': '[B', 'left': '[D', 'right': '[C', 'space': ' '}
        return key_map.get(event.name, event.name)

def stream_webcam():
    global streaming
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    while streaming:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        cv2.imshow('Webcam Feed', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            streaming = False
            break

    cap.release()
    cv2.destroyAllWindows()

def start_stream():
    global webcam_thread, streaming
    if not streaming:
        streaming = True
        webcam_thread = threading.Thread(target=stream_webcam)
        webcam_thread.start()

def stop_stream():
    global streaming
    if streaming:
        streaming = False
        webcam_thread.join()

while True:
    key = getKey()
    if key == 'e':
        print("Stopping")
        break
    if key == 'v':
        if streaming:
            stop_stream()
        else:
            start_stream()
    else:
        identify_key = actions.get(keyboard_cmds.get(key, 80), my_pluto.reset)
        identify_key()

# Ensure the webcam thread is properly closed
if webcam_thread and webcam_thread.is_alive():
    stop_stream()
