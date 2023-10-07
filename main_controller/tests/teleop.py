#keyboard teleop to control a robot linear and angular speed and send values to serial

from pynput import keyboard
import serial
import time
import argparse
# have the serial reades as an async thread
from threading import Thread


# receive the ACM number as an argument
parser = argparse.ArgumentParser()
# make optional
parser.add_argument("--acm", help="ACM number of the serial port", type=int, default=0)
args = parser.parse_args()

ACM = args.acm

#serial port
ser = serial.Serial(f"/dev/teensy", 115200, timeout=5)
ser.flush()
def on_press(key):
    global ACM
    global ser
    x = 0
    y = 0
    z = 0
    th = 0
    speed = 0.8 # adjust as needed
    val = ''
    if key == keyboard.Key.space:
        key.char = 'X'
    if key == keyboard.Key.up:
        key.char = 'U'
    if key == keyboard.Key.down:
        key.char = 'D'
    if key == keyboard.Key.shift:
        key.char = 'S'
    if key == keyboard.Key.enter:
        key.char = 'I'
    if key ==keyboard.Key.backspace:
        key.char = '/'
    try:
        # space to stop
        val = key.char
    except AttributeError as e:
        print(e)
        print("Invalid key pressed")
    #send values to serial
    try:
        ser.write(str.encode(val))
    except Exception as e:
        # try reconnecting to either ACM0 or ACM1
        ACM = 0 if ACM == 1 else 1
        print(e)
        print("Trying to reconnect to serial")
        ser = serial.Serial(f"/dev/ttyACM{ACM}", 115200, timeout=5)
        ser.flush()
    #print(f"\nx = {x}, y = {y}, z = {z}, th = {th}, data = {val}")
    print(f"------- Sent data = {val} -----------")

def on_release(key):
    if key == keyboard.Key.esc:
        return False

def serialReadThread():
    # work as an Arduino Serial Monitor
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)

#keyboard teleop
while True:
    # start the serial read thread
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
    

'''
from pynput import keyboard
import serial
import time

#serial port
ser = serial.Serial("/dev/ttyACM0", 115200, timeout=5)
ser.flush()
def on_press(key):
    x = 0
    y = 0
    z = 0
    th = 0
    speed = 0.8 # adjust as needed
    val = ''
    
    #
    if key == keyboard.Key.up:
        x = speed
        val = 'w'
    elif key == keyboard.Key.down:
        x = -speed
        val = 'x'
    elif key == keyboard.Key.left:
        z = speed
        val = 'a'
    elif key == keyboard.Key.right:
        z = -speed
        val = 'd'
    elif key == keyboard.Key.space:
        th = speed
        val = 'q'
    elif key == keyboard.Key.shift:
        th = -speed
        val = 'e'
    elif key == keyboard.Key.esc:
        val = 's'
    else:
        return
    
    #send values to serial
    ser.write(str.encode(val))

    print(x,z,th)

def on_release(key):
    if key == keyboard.Key.esc:
        return False

#keyboard teleop
while True:
    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()
'''