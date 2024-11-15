import rospy
import serial
import time
from std_msgs.msg import Int32MultiArray

def UARTReceive():
    try:
        ser.port = "/dev/ttyTHS0"
        ser.baudrate = 9600
        ser.timeout = 1  # seconds
        ser.open()
    except serial.SerialException as e:
        rospy.logerr("Unable to open port: {}".format(e))
        exit(-1)

    if ser.is_open:
        rospy.loginfo("Serial port initialized")
    else:
        exit(-1)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            data = ser.read(5)  # 读取5个字节
            if len(data) == 5 and data[0] == 0xfd and data[4] == 0xdf:
                data1, data2, data3 = data[1], data[2], data[3]
                rospy.loginfo("Received data: {}, {}, {}".format(data1, data2, data3))
                msg = Int32MultiArray(data=[data1, data2, data3])
                uart_pub.publish(msg)
            else:
                rospy.logwarn("Invalid data received: {}".format(data))

if __name__ == "__main__":
    rospy.init_node("uart_receive_node")
    uart_pub = rospy.Publisher("uart_data", Int32MultiArray, queue_size=10)

    ser = serial.Serial()

    UARTReceive()
    
    rospy.spin()

