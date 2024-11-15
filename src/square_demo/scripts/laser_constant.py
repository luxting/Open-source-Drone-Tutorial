import Jetson.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32


led_pin = 7
GPIO.setmode(GPIO.BOARD)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.output(led_pin, GPIO.LOW)
while 1:
  a=1
