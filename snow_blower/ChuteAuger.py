import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy, Range
import RPi.GPIO as GPIO
import time
import sys

Dir = [
    'forward',
    'backward',
]

class ChuteAuger:
    def __init__(self):
        self.EN_AUGER = 36 #16
        self.IN1 = 32 #12
        self.IN2 = 35 #19

        self.EN_CHUTE = 38 #20
        self.IN3 = 40 #21
        self.IN4 = 37 #26

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.EN_AUGER,GPIO.OUT)
        GPIO.setup(self.IN1,GPIO.OUT)
        GPIO.setup(self.IN2,GPIO.OUT)
        GPIO.output(self.IN1,GPIO.LOW)
        GPIO.output(self.IN2,GPIO.LOW)
        self.p_auger=GPIO.PWM(self.EN_AUGER,100)
        self.p_auger.start(0)

        GPIO.setup(self.EN_CHUTE,GPIO.OUT)
        GPIO.setup(self.IN3,GPIO.OUT)
        GPIO.setup(self.IN4,GPIO.OUT)
        GPIO.output(self.IN3,GPIO.LOW)
        GPIO.output(self.IN4,GPIO.LOW)
        self.p_chute=GPIO.PWM(self.EN_CHUTE,100)
        self.p_chute.start(0)

    def MotorRun(self, motor, index):
        if(motor == 0):
            if(index == Dir[0]):
                self.p_auger.ChangeDutyCycle(75)
                GPIO.output(self.IN1,GPIO.HIGH)
                GPIO.output(self.IN2,GPIO.LOW)
                time.sleep(8)
                self.p_auger.ChangeDutyCycle(0)
            else:
                self.p_auger.ChangeDutyCycle(75)
                GPIO.output(self.IN1,GPIO.LOW)
                GPIO.output(self.IN2,GPIO.HIGH)
                time.sleep(8)
                self.p_auger.ChangeDutyCycle(0)
        else:
            if(index == Dir[0]):
                self.p_chute.ChangeDutyCycle(75)
                GPIO.output(self.IN3,GPIO.HIGH)
                GPIO.output(self.IN4,GPIO.LOW)
                time.sleep(6)
                self.p_chute.ChangeDutyCycle(0)
            else:
                self.p_chute.ChangeDutyCycle(75)
                GPIO.output(self.IN3,GPIO.LOW)
                GPIO.output(self.IN4,GPIO.HIGH)
                time.sleep(6)
                self.p_chute.ChangeDutyCycle(0)

    def MotorStop(self, motor):
        if (motor == 0):
            self.p_auger.ChangeDutyCycle(0)
        else:
            self.p_chute.ChangeDutyCycle(0)