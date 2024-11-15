#include <ros/ros.h>
#include "api_library.h"
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandTOL.h>

#define takeoff_HGT 1.3
#define pi 3.141592653589


bool land_flag = false;
float land_x = 0;
float land_y = 0;
float land_yaw = 0;
int mission_num = 0;

mavros_msgs::RCIn rc;
int rc_value;
void rc_cb(const mavros_msgs::RCIn::ConstPtr &msg) // 遥控器通道值回调函数
{
        rc = *msg;
        rc_value = rc.channels[4]; // 订阅第五通道
}

int main(int argc, char **argv)
{
        setlocale(LC_ALL, "");

        ros::init(argc, argv, "precise_drop");

        ros::NodeHandle nh;

        ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

        ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("mavros/local_position/odom", 10, local_pos_cb);

        ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("object_position", 100, object_pos_cb);

        ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

        ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

        ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

        ros::Subscriber rc_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rc_cb); // 订阅遥控器通道pwm

        ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
        
        ros::Publisher  goods_pub = nh.advertise<std_msgs::Int32>("box_info", 10);

        ros::Rate rate(20);

        while (ros::ok() && !current_state.connected)
        {
                ros::spinOnce();
                rate.sleep();
        }
        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.x = init_position_x_take_off;
        setpoint_raw.position.y = init_position_y_take_off;
        setpoint_raw.position.z = takeoff_HGT;
        setpoint_raw.yaw = init_yaw;

        for (int i = 100; ros::ok() && i > 0; --i)
        {
                mavros_setpoint_pos_pub.publish(setpoint_raw);
                ros::spinOnce();
                rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        mavros_msgs::CommandTOL land_cmd;
        // land_cmd.request.yaw = 0;
        land_cmd.request.latitude = 0;
        land_cmd.request.longitude = 0;
        land_cmd.request.altitude = 0;

        std_msgs::Int32 goods;

        ros::Time last_request = ros::Time::now();

        // lib_pwm_control(20, 20);
        // ctrl_pwm_client.call(lib_ctrl_pwm);
        while(ros::ok()&& rc_value > 900 && rc_value < 1150)
        {
                ROS_INFO("waiting");
		ros::spinOnce();
                rate.sleep();

        }

        while (ros::ok() &&rc_value > 1400)
        {
                if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                        if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                        {
                                ROS_INFO("Offboard enabled");
                        }
                        last_request = ros::Time::now();
                        flag_init_position = false;
                }
                else
                {
                        if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                        {
                                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                                {
                                        ROS_INFO("Vehicle armed");
                                }
                                last_request = ros::Time::now();
                                flag_init_position = false;
                        }
                }

                // 1、添加高度判断，使得无人机跳出模式切换循环
                if (fabs(local_pos.pose.pose.position.z - takeoff_HGT) < 0.05)
                {
                        if (ros::Time::now() - last_request > ros::Duration(4.0))
                        {
                                mission_num = 1;
                                break;
                        }
                }
                // 2、添加时间判断，使得无人机跳出模式切换循环
                if (ros::Time::now() - last_request > ros::Duration(15.0))
                {
                        mission_num = 1;
                        break;
                }
                // 此处添加是为增加无人机的安全性能，在实际测试过程中，采用某款国产的GPS和飞控，气压计和GPS定位误差极大，
                // 导致了无人机起飞后直接飘走，高度和位置都不正常，无法跳出模式循环，导致遥控且无法接管
                // 因此增加了时间判断，确保无人机在切入offboard模式和解锁后，确保任何情况下，8秒后遥控器都能切入其他模式接管无人机
                // 注意：一定要确定GPS和飞控传感器都是正常的
                // 注意：一定要确定GPS和飞控传感器都是正常的
                // 注意：一定要确定GPS和飞控传感器都是正常的
                // 注意：一定要确定GPS和飞控传感器都是正常的
                // 注意：一定要确定GPS和飞控传感器都是正常的
                mission_pos_cruise(0, 0, takeoff_HGT, init_yaw, 0.5);
                mavros_setpoint_pos_pub.publish(setpoint_raw);
                ros::spinOnce();
                rate.sleep();
        }
        float targetA[9][4] =
            {
                {0, 0.75, takeoff_HGT, init_yaw},
                {0, 1.25, takeoff_HGT, init_yaw},
                {0, 1.75, takeoff_HGT, init_yaw},
                {0, 1.75, takeoff_HGT - 0.4, init_yaw},
                {0, 1.25, takeoff_HGT - 0.4, init_yaw},
                {0, 0.75, takeoff_HGT - 0.4, init_yaw},
                {0, -0.25, takeoff_HGT - 0.4, init_yaw},
                {1.50, -0.25, takeoff_HGT - 0.4, init_yaw},
                {1.50, -0.25, takeoff_HGT - 0.4, init_yaw + pi}};
        int goodsA[2][6]={
                {13,12,11,14,15,16},
                {24,25,26,23,22,21}
        };
        float targetB[7][4] =
            {
                {1.50, 0.75, takeoff_HGT - 0.4, init_yaw + pi}, // B4
                {1.50, 1.25, takeoff_HGT - 0.4, init_yaw + pi}, // B5
                {1.50, 1.75, takeoff_HGT - 0.4, init_yaw + pi}, // B6
                {1.50, 1.75, takeoff_HGT, init_yaw + pi}, // B3
                {1.50, 1.25, takeoff_HGT, init_yaw + pi}, // B2
                {1.50, 0.75, takeoff_HGT, init_yaw + pi}, // B1
                {1.50, 2.50, takeoff_HGT, init_yaw + pi}, // B至降落
            };

        bool flag_arrive = false;
        while (ros::ok() && rc_value > 1400 )
        {
                printf("mission_num = %d\r\n", mission_num);
                switch (mission_num)
                {
                case 1:
                        static int flag111 = 0;
                        while(ros::ok()&& rc_value > 1400&&flag111<3)
                        {
                                if (mission_pos_cruise(targetA[flag111][0], targetA[flag111][1], targetA[flag111][2], targetA[flag111][3], 0.05))
                                {
                                        goods.data = goodsA[0][flag111];
                                        if (time_record_func(3.0, ros::Time::now()))
                                        {
                                                flag111 += 1;
                                                if(flag111 == 3)
                                                {
                                                        mission_num = 98;
                                                        break;
                                                }
                                        }
                                }
                                mavros_setpoint_pos_pub.publish(setpoint_raw);
                                goods_pub.publish(goods);
                                ros::spinOnce();
                                rate.sleep();
                        }    
                        break;
                        // // 通过定点飞行，发布无人机的目标位置
                        // if (mission_pos_cruise(1, 0, takeoff_HGT, init_yaw, 0.3))
                        // {
                        //         if (time_record_func(3.0, ros::Time::now()))
                        //         {
                        //                 mission_num = 2;
                        //         }
                        // }
                        // break;
                case 98:
                        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
                        setpoint_raw.coordinate_frame = 1;
                        setpoint_raw.position.x = targetA[flag111][0];
                        setpoint_raw.position.y = targetA[flag111][1];
                        setpoint_raw.position.z = targetA[flag111][2];
                        setpoint_raw.yaw = targetA[flag111][3];
                        if(abs(local_pos.pose.pose.position.z-targetA[flag111][2])<0.05)
                        {
                                goods.data = goodsA[0][flag111];
                               if (time_record_func(3.0, ros::Time::now()))
                                {
                                        mission_num = 97;
                                } 
                        }
                        break;
                case 97:
                       while(ros::ok()&& rc_value > 1400&&flag111<6)
                        {
                                if (mission_pos_cruise(targetA[flag111][0], targetA[flag111][1], targetA[flag111][2], targetA[flag111][3], 0.05))
                                {
                                        goods.data = goodsA[0][flag111];
                                        if (time_record_func(3.0, ros::Time::now()))
                                        {
                                                flag111 += 1;
                                                if(flag111 == 6)
                                                {
                                                        mission_num = 99;
                                                        break;
                                                }
                                        }
                                }
                                mavros_setpoint_pos_pub.publish(setpoint_raw);
                                goods_pub.publish(goods);
                                ros::spinOnce();
                                rate.sleep();
                        }    
                        break;
                case 99:
                        if (mission_pos_cruise(targetA[flag111][0], targetA[flag111][1], targetA[flag111][2], targetA[flag111][3], 0.05))
                        {
                                if (time_record_func(3.0, ros::Time::now()))
                                {
                                        flag111 +=1;
                                        mission_num = 2;
                                }
                        }
                        break;
                case 2:
                        if (mission_pos_cruise(targetA[flag111][0], targetA[flag111][1], targetA[flag111][2], targetA[flag111][3], 0.05))
                        {
                                if (time_record_func(3.0, ros::Time::now()))
                                {
                                        flag111 += 1;
                                        mission_num = 3;
                                }
                        }

                        break;
                case 3:
                        if (mission_yaw_cruise(targetA[flag111][2], targetA[flag111][3]))
                        {
                                if (time_record_func(1.0, ros::Time::now()))
                                {
                                        mission_num = 4;
                                }
                        }
                        break;
                case 4:
                        static int flag222 = 0;
                        while (ros::ok() && rc_value > 1400 && flag222 < 3)
                        {
                                if (mission_pos_cruise(targetB[flag222][0], targetB[flag222][1], targetB[flag222][2], targetB[flag222][3], 0.05))
                                {
                                        goods.data = goodsA[1][flag222];
                                        if (time_record_func(3.0, ros::Time::now()))
                                        {
                                                flag222 += 1;
                                                if (flag222 == 3)
                                                {
                                                        mission_num = 96;
                                                        break;
                                                }
                                        }
                                }
                                mavros_setpoint_pos_pub.publish(setpoint_raw);
                                goods_pub.publish(goods);
                                ros::spinOnce();
                                rate.sleep();
                        }
                        
                        break;
                case 96:
                        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
                        setpoint_raw.coordinate_frame = 1;
                        setpoint_raw.position.x = targetB[flag222][0];
                        setpoint_raw.position.y = targetB[flag222][1];
                        setpoint_raw.position.z = targetB[flag222][2];
                        setpoint_raw.yaw = targetB[flag222][3];
                        if(abs(local_pos.pose.pose.position.z-targetB[flag222][2])<0.05)
                        {
                                goods.data = goodsA[0][flag222];
                               if (time_record_func(3.0, ros::Time::now()))
                                {
                                        mission_num = 95;
                                } 
                        }
                        break;
                case 95:
                        while(ros::ok()&& rc_value > 1400&&flag222<6)
                        {
                                if (mission_pos_cruise(targetB[flag222][0], targetB[flag222][1], targetB[flag222][2], targetB[flag222][3], 0.05))
                                {
                                        goods.data = goodsA[0][flag222];
                                        if (time_record_func(3.0, ros::Time::now()))
                                        {
                                                flag222 += 1;
                                                if(flag222 == 6)
                                                {
                                                        mission_num = 5;
                                                        break;
                                                }
                                        }
                                }
                                mavros_setpoint_pos_pub.publish(setpoint_raw);
                                goods_pub.publish(goods);
                                ros::spinOnce();
                                rate.sleep();
                        }    
                        break;
                case 5:
                        if (mission_pos_cruise(targetB[flag222][0], targetB[flag222][1], targetB[flag222][2], targetB[flag222][3], 0.05))
                        {
                                if (time_record_func(3.0, ros::Time::now()))
                                {
                                        flag222 += 1;
                                        mission_num = 6;
                                }
                        }
                        break;
                case 6:
                        flag_arrive = true;
                        ROS_INFO("进入自动降落模式,任务结束");
                        ros::spinOnce();
                        break;
                }
                // 此处使用高度低于起飞初始值，可以利用位置环的PID控制，有效的抵消风的影响
                if (flag_arrive == true)
                {
                        if(land_flag == false)
                        {
                                // land_x = local_pos.pose.pose.position.x;
                                // land_y = local_pos.pose.pose.position.y;
                                land_yaw = yaw;
                                land_flag = true;
                        }
                        
                        
                        printf("23456\r\n");
                        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
                        setpoint_raw.coordinate_frame = 1;
                        setpoint_raw.position.x = 1.5;
                        setpoint_raw.position.y = 2.5;
                        setpoint_raw.position.z = init_position_z_take_off - 0.5;
                        setpoint_raw.yaw        = land_yaw;
                        land_cmd.request.yaw = land_yaw;
                        if(time_record_func(10.0, ros::Time::now()))
                        {
                        while (ros::ok())
                        {
                                ROS_INFO("tring to land");
                                while (!(land_client.call(land_cmd) &&
                                         land_cmd.response.success))
                                {
                                        // local_pos_pub.publish(pose);
                                        ROS_INFO("land");
                                        ros::spinOnce();
                                        rate.sleep();
                                }
                        }
                        }
                }
                mavros_setpoint_pos_pub.publish(setpoint_raw);
                ros::spinOnce();
                rate.sleep();
        }
        return 0;
}
