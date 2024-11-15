#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
serial::Serial ser;
int num[3] = {3,4,17};
void UARTServo(int num[]) {
    int data1 = num[0];
    int data2 = num[1];
    int data3 = num[2];
    //int data4 = num[3];
    uint8_t cmd[5] = {0xfd, static_cast<uint8_t>(data1), static_cast<uint8_t>(data2), static_cast<uint8_t>(data3), 0xdf};
    ser.write(cmd, 6);
    ros::Duration(0.05).sleep();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "uart_node");
    ros::NodeHandle nh;

    try {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    } catch (serial::IOException& e) {
        ROS_ERROR_STREAM("Unable to open port: " << e.what());
        return -1;
    }

    if (ser.isOpen()) {
        ROS_INFO_STREAM("Serial port initialized");
    } else {
        return -1;
    }
    // ros::Rate loop_rate(2);
    UARTServo(num);

    ros::spin();
    // loop_rate.sleep();
    return 0;
}

