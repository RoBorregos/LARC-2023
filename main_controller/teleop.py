#keyboard teleop to control a robot linear and angular speed and send values to serial

from pynput import keyboard
import serial
import time

#serial port
ser = serial.Serial("COM5", 9600, timeout=5)
ser.flush()
def on_press(key):
    x = 0
    y = 0
    z = 0
    th = 0
    speed = 0.8 # adjust as needed
    val = ''

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