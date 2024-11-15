#include <ros/ros.h>
#include <std_msgs/String.h>
#include <serial/serial.h>
#include <vector>
#include <std_msgs/Int32.h>

class SerialNode {
public:
    SerialNode() : nh_("~") {
        // 串口初始化
        serial_.setPort("/dev/ttyUSB0"); // 请根据你的设备设置串口名称
        serial_.setBaudrate(9600); // 请根据你的设备设置波特率
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);

        try {
            serial_.open();
            ROS_INFO_STREAM("Serial port opened.");
        } catch (serial::IOException &e) {
            ROS_ERROR_STREAM("Unable to open serial port.");
            ros::shutdown();
        }

        // 发布器和订阅器初始化
        pub_ = nh_.advertise<std_msgs::String>("serial_data", 10);
        sub_ = nh_.subscribe("serial_command", 10, &SerialNode::commandCallback, this);
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            if (serial_.available()) {
                std::string data = serial_.read(serial_.available());
                processIncomingData(data);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void commandCallback(const std_msgs::String::ConstPtr& msg) {
        if (serial_.isOpen()) {
            std::vector<uint8_t> frame = {0xfd}; // 帧头
            for (char c : msg->data) {
                frame.push_back(static_cast<uint8_t>(c));
            }
            frame.push_back(0xdf); // 帧尾
            serial_.write(frame);
        } else {
            ROS_ERROR_STREAM("Serial port is not open.");
        }
    }

    void processIncomingData(const std::string& data) {
        // 解析数据帧
        static std::vector<uint8_t> buffer;
        buffer.insert(buffer.end(), data.begin(), data.end());

        while (buffer.size() >= 6) { // 最小帧长度是6
            auto it = std::find(buffer.begin(), buffer.end(), 0xfd); // 查找帧头
            if (it != buffer.end()) {
                if (std::distance(it, buffer.end()) >= 6) {
                    auto end_it = std::find(it + 1, buffer.end(), 0xdf); // 查找帧尾
                    if (end_it != buffer.end()) {
                        // 提取帧数据
                        std::vector<uint8_t> frame(it + 1, end_it);
                        std::string frame_str;
                        for (auto byte : frame) {
                            frame_str += std::to_string(byte) + " ";
                        }

                        // 发布帧数据
                        std_msgs::Int32 msg;
                        msg.data = static_cast<uint8_t>(frame_str[1]);
                        pub_.publish(msg);

                        // 移除已处理的数据
                        printf(frame_str);
                        buffer.erase(buffer.begin(), end_it + 1);
                    } else {
                        // 如果没有找到帧尾，保留剩余数据以待下次处理
                        break;
                    }
                } else {
                    // 如果帧头之后的数据不足一个完整帧，保留剩余数据
                    break;
                }
            } else {
                // 如果没有找到帧头，清空缓冲区
                buffer.clear();
            }
        }
    }

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    serial::Serial serial_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "serial_node");
    SerialNode node;
    node.spin();
    return 0;
}
