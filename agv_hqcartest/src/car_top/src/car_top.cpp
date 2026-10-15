// car_top.cpp (根据你的协议改为 线速度+角速度 版)
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "car_top/car_top.h"
#include "controlcan.h"

using namespace std;

// -------------------------- 参数定义 --------------------------
VCI_BOARD_INFO pInfo;
uint8_t battery_level = 0;
uint8_t error_flag = 0;

// -------------------------- 发送函数 --------------------------
void SendSpeedToChassis(double linear_m_s, double angular_rad_s)
{
    // 单位换算
    int16_t linear_mm_s = static_cast<int16_t>(linear_m_s * 1000.0); // m/s → mm/s
    if(linear_m_s > 0.5)
    {
        linear_mm_s = 0;
    }
    int16_t angular_deg = angular_rad_s*1000.0f;     //
   //nt16_t angular_100 = static_cast<int16_t>(angular_deg * 100.0); // 放大100倍

    VCI_CAN_OBJ send[1];
    send[0].ID = 0x00000001;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 8;

    send[0].Data[0] = 0x01; // 底盘Node_ID
    send[0].Data[1] = 0x01; // 命令
    send[0].Data[2] = (uint8_t)(linear_mm_s & 0xFF);
    send[0].Data[3] = (uint8_t)((linear_mm_s >> 8) & 0xFF);
    send[0].Data[4] = (uint8_t)(angular_deg & 0xFF);
    send[0].Data[5] = (uint8_t)((angular_deg >> 8) & 0xFF);
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    VCI_Transmit(VCI_USBCAN1, 0, 0, send, 1);
    printf("[TX] Linear=%d mm/s, Angular=%d (°×100)\n", linear_mm_s, angular_deg);
}

// -------------------------- 回调函数 --------------------------
void cmd_velCallback(const geometry_msgs::Twist &msg)
{
    SendSpeedToChassis(msg.linear.x, msg.angular.z);
}

// -------------------------- 主函数 --------------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "car_top");
    ros::NodeHandle n;

    // 初始化 CAN
    if (VCI_OpenDevice(VCI_USBCAN1, 0, 0) != 1)
    {
        ROS_ERROR("Open device failed!");
        return -1;
    }

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 2;
    config.Timing0 = 0x00; // 500Kbps
    config.Timing1 = 0x1C;
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN1, 0, 0, &config) != 1 ||
        VCI_StartCAN(VCI_USBCAN1, 0, 0) != 1)
    {
        ROS_ERROR("Init CAN failed!");
        VCI_CloseDevice(VCI_USBCAN1, 0);
        return -1;
    }

    ros::Subscriber sub = n.subscribe("/cmd_vel", 10, cmd_velCallback);
    ros::Publisher pub = n.advertise<car_top::car_top>("/car_message", 10);
    ros::Rate rate(50);

    VCI_CAN_OBJ recv[50];

    while (ros::ok())
    {
        int len = VCI_Receive(VCI_USBCAN1, 0, 0, recv, 50, 0);
        if (len > 0)
        {
            for (int i = 0; i < len; i++)
            {
                if (recv[i].Data[0] == 0x02 && recv[i].Data[1] == 0x02)
                {
                    int16_t linear_fb = (int16_t)((recv[i].Data[3] << 8) | recv[i].Data[2]);
                    int16_t angular_fb = (int16_t)((recv[i].Data[5] << 8) | recv[i].Data[4]);
                   // battery_level = recv[i].Data[6];
                   // error_flag = recv[i].Data[7];

                    car_top::car_top msg;
                    msg.left_speed = linear_fb;
                    msg.right_speed = angular_fb;
                    ROS_INFO("VX = %f",msg.left_speed );
                     ROS_INFO("VZ = %f",msg.right_speed );
                   // msg.back_wheel_speed = linear_fb;
                   /// msg.turn_angle = angular_fb; // °×100
                    //msg.left_speed = linear_fb;  // 可保持兼容
                    //msg.right_speed = linear_fb;
                   // msg.battery_level = battery_level;
                   // msg.error_flag = error_flag;

                    pub.publish(msg);
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    VCI_CloseDevice(VCI_USBCAN1, 0);
    return 0;
}