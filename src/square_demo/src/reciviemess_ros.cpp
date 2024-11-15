#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <serial/serial.h>
#include <vector>

serial::Serial ser;

void UARTReceive()
{
    if (ser.isOpen() && ser.available())
    {
        std::vector<uint8_t> data(5);
        ser.read(data, 5);
        ROS_INFO("%f,%f,%f,%f,%f",data[0],data[1],data[2],data[3],data[4]);
        //ROS_WARN(data.[0]);
        //ROS_WARN(data.[4]);

        if (data.size() == 5 && data[0] == 0xfd && data[4] == 0xdf)
        {
            int data1 = data[1];
            int data2 = data[2];
            int data3 = data[3];

            ROS_INFO("Received data: %d, %d, %d", data1, data2, data3);

            std_msgs::Int32MultiArray msg;
            msg.data.push_back(data1);
            msg.data.push_back(data2);
            msg.data.push_back(data3);

            ros::NodeHandle nh;
            ros::Publisher uart_pub = nh.advertise<std_msgs::Int32MultiArray>("uart_data", 10);
            uart_pub.publish(msg);
        }
        else
        {
            ROS_WARN("Invalid data received");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uart_receive_node");
    ros::NodeHandle nh;

    try
    {
        ser.setPort("/dev/ttyTHS0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        UARTReceive();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
