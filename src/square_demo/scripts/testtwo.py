#!/usr/bin/env python

import rospy
import cv2
from pyzbar.pyzbar import decode
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import serial

class QRCodeReader:
    def __init__(self, mapping_file, serial_port="/dev/ttyUSB0", baudrate=9600):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.qr_code_pub = rospy.Publisher("/qr_code_data", String, queue_size=10)
        self.serial_port = serial.Serial(port=serial_port, baudrate=baudrate, timeout=1)
        self.mapping = self.load_mapping(mapping_file)
        self.qr_code_detected = False  # Flag to ensure only one detection

    def load_mapping(self, mapping_file):
        mapping = {}
        with open(mapping_file, 'r') as file:
            for line in file:
                key, value = line.strip().split(' ')
                mapping[key] = value
        rospy.loginfo("Loaded mapping: {}".format(mapping))
        return mapping

    def image_callback(self, data):
        if self.qr_code_detected:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        decoded_objects = decode(cv_image)
        for obj in decoded_objects:
            qr_code_data = obj.data.decode("utf-8").strip()
            rospy.loginfo("Detected QR Code: '{}'".format(qr_code_data))
            for key, value in self.mapping.items():
                if qr_code_data == value:
                    rospy.loginfo("Mapped Value: {}".format(key))
                    self.qr_code_pub.publish(key)
                    
                    # Send data to serial port
                    try:
                        data4 = int(key[:2], 16)
                        cmd = bytearray([0xfd, 0, data4, 0xdf])
                        self.serial_port.write(cmd)
                        rospy.loginfo("Sent data over serial: {}".format(cmd))
                        self.qr_code_detected = True  # Mark as detected
                        break
                    except ValueError:
                        rospy.logerr("Invalid mapped value for serial communication")
            else:
                rospy.logwarn("QR code data not found in mapping: '{}'".format(qr_code_data))

        cv2.waitKey(3)

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

