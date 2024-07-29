import os
import keyboard as kb
from Pluto import pluto
import time
from threading import Thread

my_pluto = pluto()

def pos_hold():
    global run_status
    my_pluto.activate_position_hold()
    try:
        while run_status:
            my_pluto.update_position_hold()
    except KeyboardInterrupt:
        run_status = False
    

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
    211:my_pluto.flip,
    212:my_pluto.get_gps,
    213: pos_hold,
}

    

def clean_exit():
    global run_status
    run_status = False
    try:
        print("Disarming...")
        my_pluto.disarm()
        time.sleep(2)
        print("disconnected..")
        my_pluto.disconnect()
    except Exception as e:
        print("Exception during clean exit : {e}")
    finally:
        print("Exiting")
        exit()

keyboard_cmds = {  # dictionary containing the key pressed and value associated with it
    '[A': 10, '[D': 30, '[C': 40, 'w': 50, 's': 60, ' ': 70, 'r': 80, 't': 90,
    'p': 100, '[B': 110, 'n': 120, 'q': 130, 'e': 140, 'a': 150, 'd': 160,
    '+': 15, '1': 25, '2': 30, '3': 35, '4': 45, 'c': 200, 'x': 210 , 'f':211 , 'g': 212, 'p':213,
}
run_status = True
def getKey():
    global run_status
    # print("started")
    event = kb.read_event()
    if event.event_type == kb.KEY_DOWN:
        key_map = {'up': '[A', 'down': '[B', 'left': '[D', 'right': '[C', 'space': ' '}
        return key_map.get(event.name, event.name)

def keyList():
    global run_status
    while run_status:
        key = getKey()
        if key == 'e':
            run_status = False
            print("stopping")
            break
        identify_key = actions.get(keyboard_cmds.get(key, 80), my_pluto.reset)
        identify_key()

if __name__ == "__main__":
    listener_thread = Thread(target=keyList)
    listener_thread.start()
    try:
        while run_status:
            time.sleep(0.1)
    except KeyboardInterrupt:
        run_status = False
        print("KeyInterrupt")
        clean_exit()
    listener_thread.join()