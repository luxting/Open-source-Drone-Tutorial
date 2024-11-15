unf -8

#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include<iostream>

//创建串口对象 
serial::Serial ser;
//创建获取的距离信息保存的数据变量
std_msgs::Float64 distance;
std_msgs::Int32 data3;

int main (int argc, char** argv){
	//创建ros节点
     ros::init(argc, argv, "good_info");
     ros::NodeHandle nh;
     //创建发布者
     ros::Publisher distance_pub=nh.advertise<std_msgs::Int32>("good_info",1000);
     //打开串口设备
     try
    {
        ser.setPort("/dev/ttyUSB0");
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }
 
    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port open");
    }else{
       return -1;
    }
    //设置运行频率
    ros::Rate loop_rate(50);
    while(ros::ok()){
    //读取串口数据
	 size_t n=ser.available();
         if(n!=0)
	 {
	 //buffer长度可以根据自己的通信协议来修改，可以改大一点如100
	     unsigned char buffer[5]={0};
	     n=ser.read(buffer,n);
	     for(int i=0;i<n;i++)
	     {
	     	std::cout<<std::hex<<(buffer[i]&0xff)<<" ";
	     }
	     std::cout<<std::endl;
	     //此处为通信协议解析，使用者根据自己的实际情况就行修改
        if ((buffer[0] & 0xff) == 0xfd && (buffer[n-1] & 0xff) == 0xdf) {
            // 提取 buffer[1] 和 buffer[2] 的值并将其转换为十进制
            std::cout << "datas1: " << buffer[1] << std::endl;
            std::cout << "datas2: " << buffer[2] << std::endl;
            int datas1 = buffer[1];
            int datas2 = buffer[2];
            datas1 = datas1 - 32;
            datas2 = datas2  - 32;
            // 打印结果
            std::cout << "datas1: " << datas1 << std::endl;
            std::cout << "datas2: " << datas2 << std::endl;
            data3.data=datas1 * 10 +datas2;
        }

	     
	}
	//发布std_msgs::Float64类型的距离数据
        distance_pub.publish(data3);
        loop_rate.sleep();
   }
 }
