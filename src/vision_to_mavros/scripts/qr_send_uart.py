#!/usr/bin/env python

import rospy
import cv2
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray, Int32
import serial
import time
import binascii



#####################


#########

class QRCodeReader:
    def __init__(self, mapping_file, serial_port="/dev/ttyTHS0", baudrate=9600):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.qr_code_pub = rospy.Publisher("/qr_code_data", String, queue_size=10)
        self.qr_code_position_pub = rospy.Publisher("/qr_code_position", Float32MultiArray, queue_size=10)
        self.mapping = self.load_mapping(mapping_file)
        self.box_info_sub = rospy.Subscriber("box_info", Int32, self.box_info_callback)
        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
        
        self.current_box_info = 0
        self.current_qr_code_data = "0"
        self.previous_box_info = 0
        self.previous_qr_code_data = "0"



    def load_mapping(self, mapping_file):
        mapping = {}
        with open(mapping_file, 'r') as file:
            for line in file:
                key, value = line.strip().split(': ')
                mapping[value] = key
        return mapping

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        decoded_objects = decode(cv_image)
        height, width, _ = cv_image.shape
        center_x, center_y = width // 2, height // 2

        for obj in decoded_objects:
            self.current_qr_code_data = obj.data.decode("utf-8")
            rospy.loginfo("Detected QR Code: {}".format(self.current_qr_code_data))
            if self.current_qr_code_data in self.mapping:
                mapped_value = self.mapping[self.current_qr_code_data]
                rospy.loginfo("Mapped Value: {}".format(mapped_value))
                self.qr_code_pub.publish(mapped_value)

            points = obj.polygon
            if points:
                avg_x = sum([point.x for point in points]) / len(points)
                avg_y = sum([point.y for point in points]) / len(points)
                position = Float32MultiArray(data=[avg_x - center_x, avg_y - center_y])
                rospy.loginfo("QR Code Position: x = {}, y = {}".format(position.data[0], position.data[1]))
                self.qr_code_position_pub.publish(position)

            for point in points:
                cv2.circle(cv_image, (point.x, point.y), 5, (0, 255, 0), -1)
            cv2.circle(cv_image, (center_x, center_y), 5, (255, 0, 0), -1)

        #cv2.imshow("QR Code Reader", cv_image)
        cv2.waitKey(3)

        self.check_and_send_serial()

    def box_info_callback(self, data):
        self.current_box_info = data.data
        self.check_and_send_serial()

    def check_and_send_serial(self):
        if (self.current_box_info != self.previous_box_info) and (self.current_qr_code_data != self.previous_qr_code_data):
            try:
                # Find the mapped value for the current QR code data
                mapped_value = self.mapping.get(self.current_qr_code_data, "0")
                rospy.loginfo("Mapped Value: {}".format(mapped_value))
                data4 =int(mapped_value[0:2])
                # Convert the mapped value to an integer if it's numeric
                #data3 = int(mapped_value) if mapped_value.isdigit() else 0
                tens = self.current_box_info // 10
                units = self.current_box_info % 10
                cmd = bytearray([0xfd, tens, units, data4, 0xdf])
                self.serial_port.write(cmd)
                rospy.loginfo("Sent data over serial: {}".format(cmd))
                time.sleep(0.05)
                self.previous_box_info = self.current_box_info
                self.previous_qr_code_data = self.current_qr_code_data
            except ValueError:
                rospy.logerr("Invalid QR code data for serial communication")

def main():
    rospy.init_node('qr_code_reader', anonymous=True)
    mapping_file = rospy.get_param('~mapping_file', 'qr_code_data.txt')
    qr_code_reader = QRCodeReader(mapping_file)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down QR Code Reader node.")
    cv2.destroyAllWindows()
    qr_code_reader.serial_port.close()

if __name__ == '__main__':
    main()
