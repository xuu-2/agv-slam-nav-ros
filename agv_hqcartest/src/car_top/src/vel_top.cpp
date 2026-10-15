#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "car_top/car_top.h"
#include "controlcan.h"

using namespace std;

#define PI 3.141592653

VCI_BOARD_INFO pInfo; // 设备信息结构体

// 全局状态变量
int16_t linear_fb_mm = 0;      // 反馈线速度 mm/s
int16_t angular_fb_deg100 = 0; // 反馈角速度 °×100
uint8_t battery_level = 0;     // 电量（百分比）
uint8_t error_flag = 0;        // 状态字节

// =============================================================
// 发送函数：根据线速度(m/s) + 角速度(rad/s) 发送给底盘
// =============================================================
void SendSpeedToChassis(double linear_m_s, double angular_rad_s)
{
    // 单位换算
    int16_t linear_mm_s = static_cast<int16_t>(linear_m_s * 1000.0); // m/s → mm/s
    double angular_deg = angular_rad_s * 180.0 / PI;                 // rad/s → °/s
    int16_t angular_100 = static_cast<int16_t>(angular_deg * 100.0); // 放大100倍

    // 组帧
    VCI_CAN_OBJ send[1];
    send[0].ID = 0x00000001; // 目标CAN ID
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 8;

    send[0].Data[0] = 0x01; // Node_ID 底盘
    send[0].Data[1] = 0x01; // 命令：速度角度设置
    send[0].Data[2] = (uint8_t)(linear_mm_s & 0xFF);
    send[0].Data[3] = (uint8_t)((linear_mm_s >> 8) & 0xFF);
    send[0].Data[4] = (uint8_t)(angular_100 & 0xFF);
    send[0].Data[5] = (uint8_t)((angular_100 >> 8) & 0xFF);
    send[0].Data[6] = 0x00;
    send[0].Data[7] = 0x00;

    if (VCI_Transmit(VCI_USBCAN1, 0, 0, send, 1) == 1)
    {
        printf("[TX] Linear=%d mm/s, Angular=%d (°×100)\n", linear_mm_s, angular_100);
    }
    else
    {
        printf("[TX] Failed!\n");
    }
}

// =============================================================
// 档位设置函数
// =============================================================
void SetGear(int gear)
{
    VCI_CAN_OBJ send[1];
    send[0].ID = 0x00000001;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 8;

    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x03;
    send[0].Data[2] = (uint8_t)gear; // 0x00低速 0x01中速 0x02高速
    for (int i = 3; i < 8; i++)
        send[0].Data[i] = 0x00;

    VCI_Transmit(VCI_USBCAN1, 0, 0, send, 1);
}

// =============================================================
// 刹车力度设置函数
// =============================================================
void SetBrek(int brek)
{
    VCI_CAN_OBJ send[1];
    send[0].ID = 0x00000001;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 8;

    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x04;
    send[0].Data[2] = (uint8_t)brek;
    for (int i = 3; i < 8; i++)
        send[0].Data[i] = 0x00;

    VCI_Transmit(VCI_USBCAN1, 0, 0, send, 1);
}

// =============================================================
// 灯带设置函数（可选）
// =============================================================
void SetLed(unsigned char mode, unsigned char light, unsigned char delay, unsigned char R, unsigned char G, unsigned char B)
{
    VCI_CAN_OBJ send[1];
    send[0].ID = 0x00000001;
    send[0].SendType = 0;
    send[0].RemoteFlag = 0;
    send[0].ExternFlag = 0;
    send[0].DataLen = 8;

    send[0].Data[0] = 0x01;
    send[0].Data[1] = 0x05;
    send[0].Data[2] = mode;
    send[0].Data[3] = light;
    send[0].Data[4] = delay;
    send[0].Data[5] = R;
    send[0].Data[6] = G;
    send[0].Data[7] = B;

    VCI_Transmit(VCI_USBCAN1, 0, 0, send, 1);
}

// =============================================================
// ROS 回调：订阅速度命令
// =============================================================
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    SendSpeedToChassis(twist_aux.linear.x, twist_aux.angular.z);
}

// =============================================================
// 主函数
// =============================================================
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_top");
    ros::NodeHandle n;

    printf(">> [vel_top] Start (protocol: Byte2–3 Linear, Byte4–5 Angular)\n");

    if (VCI_OpenDevice(VCI_USBCAN1, 0, 0) != 1)
    {
        printf(">> Open device failed!\n");
        return -1;
    }

    if (VCI_ReadBoardInfo(VCI_USBCAN1, 0, &pInfo) == 1)
        printf(">> Get device info success!\n");

    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 2;
    config.Timing0 = 0x00; // 500Kbps
    config.Timing1 = 0x1C;
    config.Mode = 0;

    if (VCI_InitCAN(VCI_USBCAN1, 0, 0, &config) != 1)
    {
        printf(">> Init CAN failed\n");
        return -1;
    }
    if (VCI_StartCAN(VCI_USBCAN1, 0, 0) != 1)
    {
        printf(">> Start CAN failed\n");
        return -1;
    }

    ros::Subscriber cmd_sub = n.subscribe("/cmd_vel", 100, cmd_velCallback);
    ros::Publisher pub = n.advertise<car_top::car_top>("/car_message", 10);
    ros::Rate rate(50);

    SetGear(0); // 默认低速档
    sleep(0.1);
    SetBrek(0);
    sleep(0.1);
    SetLed(1, 100, 10, 0, 255, 0);

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
                    linear_fb_mm = (int16_t)((recv[i].Data[3] << 8) | recv[i].Data[2]);
                    angular_fb_deg100 = (int16_t)((recv[i].Data[5] << 8) | recv[i].Data[4]);
                    battery_level = recv[i].Data[6];
                    error_flag = recv[i].Data[7];

                    double angular_rad = (angular_fb_deg100 / 100.0) * PI / 180.0;

                    car_top::car_top msg;
                    msg.back_wheel_speed = linear_fb_mm;
                    msg.turn_angle = angular_fb_deg100 / 100.0;
                    msg.left_speed = linear_fb_mm;
                    msg.right_speed = linear_fb_mm;
                    msg.battery_level = battery_level;
                    msg.error_flag = error_flag;

                    pub.publish(msg);

                    // printf("[RX] Linear=%d mm/s, Angular=%.2f° (%.3f rad/s), Battery=%d%%, Flag=%02X\n",
                    //        linear_fb_mm, angular_fb_deg100 / 100.0, angular_rad, battery_level, error_flag);
                }
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    VCI_CloseDevice(VCI_USBCAN1, 0);
    return 0;
}
