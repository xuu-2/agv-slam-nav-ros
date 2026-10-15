#ifndef _PUB_CAR_ODOM_H_
#define _PUB_CAR_ODOM_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>      // /cmd_vel
#include <nav_msgs/Odometry.h>        // /odom
#include <sensor_msgs/Imu.h>          // /sensor_imu
#include <tf/transform_broadcaster.h> // tf广播
#include <string>
#include <math.h>

// ==== 自定义消息 ====
#include "car_top/imu.h"
#include "car_top/car_top.h"

using namespace std;

// ================== 常量定义 ==================
#define PI 3.14159265358979323846f

// ========== 协方差矩阵 ==========
const double odom_pose_covariance[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3};

const double odom_pose_covariance2[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9};

const double odom_twist_covariance[36] = {
    1e-3, 0, 0, 0, 0, 0,
    0, 1e-3, 0, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e3};

const double odom_twist_covariance2[36] = {
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9};

// ================== 结构定义 ==================
struct Vel_Pos_Data
{
    float X;
    float Y;
    float Z;
};
# if 0
// ================== 主类定义 ==================
class HQ_AutoCar
{
public:
    HQ_AutoCar();               // 构造函数
    ~HQ_AutoCar();              // 析构函数
    void Hq_auto_control(void); // 主循环控制

private:
    // ===== ROS 句柄与时间 =====
    ros::NodeHandle Nh_Hq;
    ros::Time _Now, _Last_Time;

    // ===== ROS 通信对象 =====
    ros::Subscriber car_top_speed_sub; // /car_message
    ros::Subscriber imu_sub;           // /sensor_imu
    ros::Publisher odom_publisher;     // /odom
    ros::Publisher imu_publisher;      // /sensor_imu 输出

    // ===== tf =====
    tf::TransformBroadcaster odom_broadcaster;

    // ===== 位姿/速度数据 =====
    Vel_Pos_Data Robot_Pos; // x,y,yaw
    Vel_Pos_Data Robot_Vel; // v_x,v_y,w_z
    uint8_t battery_level;  // 电量
    uint8_t error_flag;     // 状态标志

    // ===== 坐标系名称 =====
    std::string odom_frame_id;
    std::string hq_robot_frame_id;
    std::string hq_gyro_frame_id;

    // ===== 内部函数 =====
    void get_car_top_callback(const car_top::car_top::ConstPtr &msg); // 接收底盘反馈
    void cmd_imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg);  // 转发IMU
    void HQ_Publish_Odom();                                           // 发布里程计
};
#endif 
#endif