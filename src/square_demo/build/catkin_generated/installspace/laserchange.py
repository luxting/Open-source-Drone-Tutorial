import Jetson.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

class BoxSubscriber:
    def __init__(self):
        self.led_pin = 7
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)
        self.subscriber = rospy.Subscriber("box_info", Int32, self.box_info_callback)
        self.last_box_info = None  # 用于存储上一次接收到的box_info

    def box_info_callback(self, data):
        # 只有当接收到的box_info与上一次不同，才调用trying函数
        if self.last_box_info != data.data:
            self.last_box_info = data.data
            self.trying()

    def trying(self):
        print("off")
        GPIO.output(self.led_pin, GPIO.LOW)
        time.sleep(1)
        GPIO.output(self.led_pin, GPIO.HIGH)

if __name__ == "__main__":
    rospy.init_node('box_subscriber_node')
    box_subscriber = BoxSubscriber()
    rospy.spin()
    # 确保脚本退出时清理GPIO设置
    GPIO.cleanup()