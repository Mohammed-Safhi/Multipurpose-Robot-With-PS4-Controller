#!/usr/bin/env python3
_author_ = 'G1'

import evdev
import ev3dev.auto as ev3
import threading
import time
from ev3dev2.console import Console
from ev3dev2.sound import Sound
from ev3dev2.sensor.lego import TouchSensor, UltrasonicSensor
from threading import Thread

#from ev3dev2.motor import SpeedRPM, SpeedPercent
#arm.on_for_rotations(SpeedPercent(75), 5)

spkr = Sound()

tr = TouchSensor()

us = UltrasonicSensor()

console = Console()
closeThread = False


us_distance = 0

tr_pressed = False



def TR():
    global us_distance
    global tr_pressed
    global closeThread
    while not closeThread:
        
        us_distance = us.distance_centimeters
        
    
        if tr.is_pressed:
            tr_pressed = True
            spkr.beep("-f 500")
            
        if(us.distance_centimeters < 30):
            spkr.beep("-f 500")
            
        elif(us.distance_centimeters < 20):
            spkr.beep("-f 600")
        
            
            
        #console.reset_console()
        #console.text_at(str(us.distance_centimeters)+"cm", column=1,row=5,reset_console=True, inverse=True)


trThread = Thread(target=TR)
trThread.start()
## Some helpers ##
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def scale(val, src, dst):
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]

def scale_stick(value):
    return scale(value,(0,255),(-1000,1000))

def dc_clamp(value):
    return clamp(value,-1000,1000)

## Initializing ##
print("Finding ps4 controller...")
devices = [evdev.InputDevice(fn) for fn in evdev.list_devices()]
ps4dev = devices[0].fn

#right_motor = ev3.LargeMotor(ev3.OUTPUT_C)
#left_motor = ev3.LargeMotor(ev3.OUTPUT_B)

#robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)

gamepad = evdev.InputDevice(ps4dev)
spkr.beep("-f 200")

forward_speed = 0
side_speed = 0
running = True
arm = ev3.LargeMotor(ev3.OUTPUT_D)
rightMotor = ev3.LargeMotor(ev3.OUTPUT_C)
leftMotor = ev3.LargeMotor(ev3.OUTPUT_B)

class MotorThread(threading.Thread):
    def _init_(self):
        self.right_motor = ev3.LargeMotor(ev3.OUTPUT_C)
        self.left_motor = ev3.LargeMotor(ev3.OUTPUT_B)
        threading.Thread._init_(self)

    def run(self):
        print("Engine running!")
        while running:
            self.right_motor.run_forever(speed_sp=dc_clamp(forward_speed+side_speed))
            self.left_motor.run_forever(speed_sp=dc_clamp(-forward_speed+side_speed))
        self.right_motor.stop()
        self.left_motor.stop()

motor_thread = MotorThread()
motor_thread.setDaemon(True)
motor_thread.start()
spSpeed = 30


def goback():
    start = time.time()
    
    global side_speed
    side_speed = -scale_stick(254)
   
    while True:
        
        
        end = time.time()
        
        t = end-start
        print(t)
        if(t > 1.5):
            side_speed = -scale_stick(123)
            break
        


for event in gamepad.read_loop():
    
    #this loops infinitely
    if tr_pressed:
        goback()
        tr_pressed = False
    
    
        
       
    if event.type == 3:  
        
        if event.code == 0:#X axis on left stick
            #print("event code 2="+str(event.value))
            if(us_distance < 30 and event.value > 124):
                #print("inside")
                forward_speed= -scale_stick(123)
                side_speed = -scale_stick(123)
                continue
            forward_speed = -scale_stick(event.value)
        if event.code == 1:         #Y axis on left stick
            print(str(event.value))
            #print(us_distance)
            if(us_distance < 30 and event.value > 124):
                #print("inside")
                side_speed = -scale_stick(123)
                forward_speed= -scale_stick(123)
                continue
                
            side_speed = -scale_stick(event.value)
        if side_speed < 100 and side_speed > -100:
            side_speed = 0
        if forward_speed < 100 and forward_speed > -100:
            forward_speed = 0

    if event.type == 1 and event.code == 305 and event.value == 1:# circle
        print("O button is pressed. Stopping.")
        running = False
        closeThread = True
        trThread.join()
        time.sleep(0.5) # Wait for the motor thread to finish
        break
    if event.type == 1 and event.code == 304 and event.value == 1: # X
        spkr.beep("-f 500")
        
    if event.type == 1 and event.code == 313 and event.value == 1:# R2
        
        arm.run_forever(speed_sp=spSpeed)
    
    if event.type == 1 and event.code == 311 and event.value == 1:# R1
        
        arm.run_forever(speed_sp=-spSpeed)
        
    if event.type == 1 and event.code == 307 and event.value == 1:# triangle
        arm.stop()
    # if tr.is_pressed:
    #     spkr.beep("-f 500")

        
    # if event.type == 1 and event.code == 308 and event.value == 1:# square
    #    pass