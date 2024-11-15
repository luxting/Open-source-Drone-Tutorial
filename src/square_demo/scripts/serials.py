#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int32

# 创建串口对象
ser = serial.Serial()
# 创建获取的距离信息保存的数据变量
distance = Int32()

def main():
    # 创建ros节点
    rospy.init_node('good_info', anonymous=True)
    # 创建发布者
    distance_pub = rospy.Publisher('good_info', Int32, queue_size=1000)
    
    # 打开串口设备
    try:
        ser.port = "/dev/ttyUSB0"
        ser.baudrate = 9600
        ser.timeout = 1
        ser.open()
    except serial.SerialException as e:
        rospy.logerr("Unable to open port")
        return
    
    if ser.is_open:
        rospy.loginfo("Serial Port open")
    else:
        return
    
    # 设置运行频率
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        # 读取串口数据
        n = ser.in_waiting
        if n != 0:
            # buffer长度可以根据自己的通信协议来修改，可以改大一点如100
            buffer = ser.read(n)
            for byte in buffer:
                print(hex(byte), end=" ")
            print()
            
            # 此处为通信协议解析，使用者根据自己的实际情况进行修改
            if (buffer[0] & 0xff) == 0xfd and (buffer[-1] & 0xff) == 0xdf:
                # 提取 buffer[1] 和 buffer[2] 的值并将其转换为十进制
                datas1 = int(chr(buffer[1]))
                datas2 = int(chr(buffer[2]))

                # 打印结果
                print("datas1:", datas1)
                print("datas2:", datas2)
                data3 = datas1 * 10 + datas2

                # 发布Int32类型的距离数据
                distance_pub.publish(data3)
                
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

