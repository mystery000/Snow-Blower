import sys
import math
import time
import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Joy, Range
from geometry_msgs.msg import Twist, Vector3

Dir = [
    'forward',
    'backward',
]

class Motor:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        self.RPWML = 15
        self.LPWML = 22
        self.R_ENL = 13
        self.L_ENL = 18
        self.RPWMR = 11
        self.LPWMR = 31
        self.R_ENR = 7
        self.L_ENR = 29
        GPIO.setup(self.RPWML, GPIO.OUT)
        GPIO.setup(self.LPWML, GPIO.OUT)
        GPIO.setup(self.L_ENL, GPIO.OUT)
        GPIO.setup(self.R_ENL, GPIO.OUT)
        GPIO.output(self.R_ENL, True)
        GPIO.output(self.L_ENL, True)
        self.rpwmL = GPIO.PWM(self.RPWML, 100)
        self.lpwmL = GPIO.PWM(self.LPWML, 100)
        self.rpwmL.ChangeDutyCycle(0) # For forward rotation
        self.lpwmL.ChangeDutyCycle(0) # For backward rotation
        self.rpwmL.start(0)
        self.lpwmL.start(0)
        GPIO.setup(self.RPWMR, GPIO.OUT)
        GPIO.setup(self.LPWMR, GPIO.OUT)
        GPIO.setup(self.L_ENR, GPIO.OUT)
        GPIO.setup(self.R_ENR, GPIO.OUT)
        GPIO.output(self.R_ENR, True)
        GPIO.output(self.L_ENR, True)
        self.rpwmR = GPIO.PWM(self.RPWMR, 100)
        self.lpwmR = GPIO.PWM(self.LPWMR, 100)
        self.rpwmR.ChangeDutyCycle(0) # For forward rotation
        self.lpwmR.ChangeDutyCycle(0) # For backward rotation
        self.rpwmR.start(0)
        self.lpwmR.start(0)
    
    def MotorRun(self, motor, index, speed):
        if speed > 100:
            return
        if(motor == 0):
            if(index == Dir[0]):
                self.rpwmL.ChangeDutyCycle(speed)
            else:
                self.lpwmL.ChangeDutyCycle(speed)
        else:
            if(index == Dir[0]):
                self.rpwmR.ChangeDutyCycle(speed)
            else:
                self.lpwmR.ChangeDutyCycle(speed)