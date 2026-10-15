#include "ros/ros.h"
#include "car_top/car_top.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include <geometry_msgs/Quaternion.h>
#include "sensor_msgs/LaserScan.h"
#include "pub_car_odom.h"
#include "actionlib_msgs/GoalID.h" 
#include "geometry_msgs/PoseStamped.h" 
#include <cmath>
sensor_msgs::Imu Mpu6050; // Instantiate an IMU object //实例化IMU对象
Vel_Pos_Data Robot_Vel; // v_x,v_y,w_z
Vel_Pos_Data Robot_Pos;
double loc_Car_linerV;
double loc_car_anglerV;
float car_top_linear_velocity;  // 车线速度
const int nearby_points = 3;
const float stop_distance = 4;
bool navi_canceled = false;
geometry_msgs::PoseStamped cached_rviz_goal;
class HQ_AutoCar
{
public:
    HQ_AutoCar();
    ~HQ_AutoCar();
    void Hq_auto_control();

private:
    ros::NodeHandle nh_;
    ros::Subscriber car_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher imu_pub_;
    ros::Subscriber senser_imu_sub_;
   ros::Publisher imu_publisher;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber scan_sub;
    ros::Publisher cancel_nac_pub;
    ros::Publisher resume_pub;
    ros::Subscriber rviz_goal_sub;

   
    
    tf::TransformBroadcaster odom_broadcaster_;
    float Sampling_Time;        // 采样时间，用于积分求位移(里程)
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string gyro_frame_id_;
    double wheel_base_;
    ros::Time _Now, _Last_Time;

    double x_, y_, theta_;
    double v_lin_, v_ang_;
    ros::Time last_time_;

    void carCallback(const car_top::car_top::ConstPtr &msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void cmd_mpu6050callback(const sensor_msgs::Imu::ConstPtr &imu_msg);
    void senser_lidar_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg);
    void rviz_goal_callback(const geometry_msgs::PoseStamped::ConstPtr&goal_msg);
    void HQ_Publish_Odom();
    void updateOdom(double dt);
   // void publishTFandOdom();
};
void HQ_AutoCar::rviz_goal_callback(const geometry_msgs::PoseStamped::ConstPtr&goal_msg)
{
    cached_rviz_goal = *goal_msg;
    ROS_INFO("x = %.2f,y = %.2f,z = %.2f",goal_msg->pose.position.x,goal_msg->pose.position.y,goal_msg->pose.position.z);
}
void HQ_AutoCar::senser_lidar_callback(const sensor_msgs::LaserScan::ConstPtr & scan_msg)
{
    const std::vector<float>& ranges = scan_msg->ranges;
    int total_points = ranges.size();
    if(total_points <= 2* nearby_points)
    {
        return;
    }
    int mid_index = total_points/2;
    int start_index = mid_index - nearby_points;
    int end_index = mid_index + nearby_points;
    std::vector<float> vaild_distances;
    for(int i = start_index;i<=end_index;++i)
    {
        float dist = ranges[i];
        if(dist >= scan_msg->range_min && dist <= scan_msg->range_max)
        {
            vaild_distances.push_back(dist);
        }
    }
    if(vaild_distances.empty())
    {
        return;
    }
    float sum = 0.0;
    for(float d:vaild_distances)
    {
        sum += d;
    }
    float avg_distance = sum/ vaild_distances.size()/2.0;
    ROS_INFO("avg_distance = %f",avg_distance);
    if(avg_distance < stop_distance)
    {

        actionlib_msgs::GoalID cancel_msg;
        cancel_nac_pub.publish(cancel_msg);
        geometry_msgs::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.linear.z = 0.0;
        cmd_vel_pub.publish(stop_cmd);
        navi_canceled = true;
        ROS_WARN("正前方平均距离<2m,已取消导航并停止运动");
        
    }else 
    {
        cached_rviz_goal.header.stamp = ros::Time::now();
        resume_pub.publish(cached_rviz_goal);
    }
}
HQ_AutoCar::HQ_AutoCar()
{
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    private_nh.param<std::string>("base_frame_id", base_frame_id_, "base_footprint");
    private_nh.param<std::string>("gyro_frame_id", gyro_frame_id_, "gyro_link");
    private_nh.param<double>("wheel_base", wheel_base_, 0.35);

    car_sub_ = nh_.subscribe("/car_message", 10, &HQ_AutoCar::carCallback, this);
    imu_sub_ = nh_.subscribe("/wit/imu", 50, &HQ_AutoCar::imuCallback, this);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 50);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/sensor_imu", 50);
    senser_imu_sub_ = nh_.subscribe("/sensor_imu",100,&HQ_AutoCar::cmd_mpu6050callback,this);
    scan_sub = nh_.subscribe("/scan",10,&HQ_AutoCar::senser_lidar_callback,this);
    cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",10);
    cancel_nac_pub = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel",10);
    rviz_goal_sub = nh_.subscribe("/move_base_simple/goal",10,&HQ_AutoCar::rviz_goal_callback,this);
    resume_pub = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    x_ = y_ = theta_ = 0.0;
    v_lin_ = v_ang_ = 0.0;
    last_time_ = ros::Time::now();

    ROS_INFO("HQ_AutoCar initialized: publishing /odom + /tf + /sensor_imu");
}

   void HQ_AutoCar::cmd_mpu6050callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
   {
   Mpu6050.linear_acceleration.x = imu_msg->linear_acceleration.x;
   Mpu6050.linear_acceleration.y = imu_msg->linear_acceleration.y;
   Mpu6050.linear_acceleration.z = imu_msg->linear_acceleration.z;


   Mpu6050.angular_velocity.x = imu_msg->angular_velocity.x;
   Mpu6050.angular_velocity.y = imu_msg->angular_velocity.y;
   Mpu6050.angular_velocity.z = imu_msg->angular_velocity.z;
}

void HQ_AutoCar::carCallback(const car_top::car_top::ConstPtr &msg)
{

    //double left_speed = msg->left_speed / 1000.0;
    //double right_speed = msg->right_speed / 1000.0;
    loc_Car_linerV = msg->left_speed/1000.0f;
    loc_car_anglerV =  msg->right_speed/1000.0f;
    //v_lin_ = (left_speed + right_speed) / 2.0;
    //v_ang_ = (right_speed - left_speed) / wheel_base_;
    //v_lin_ = msg->back_wheel_speed;
    //v_ang_ = msg->turn_angle;
    //if (std::fabs(v_lin_) < 0.001) v_lin_ = 0.0;
    //if (std::fabs(v_ang_) < 0.001) v_ang_ = 0.0;
     _Now = ros::Time::now();  // 当前时间
      if (_Last_Time.toSec() == 0)
      {
         _Last_Time = _Now; 
      }// 首次进入时进行此操作，避免积分时间过大
      Sampling_Time = (_Now - _Last_Time).toSec(); // 获取时间间隔，用于积分速度获得位移(里程)
      // 速度*时间=位移（里程计）
      Robot_Vel.X = loc_Car_linerV;
      Robot_Vel.Y = 0;

      // printf("trun_zxis = %f\r\n",turn_zxis);

      Robot_Pos.X += (Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.Y += (Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; // Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
      Robot_Pos.Z += Mpu6050.angular_velocity.z * Sampling_Time;  // 旋转角度

      // 发布odom话题 HQ_Publish_Odom();
      HQ_Publish_Odom();
      _Last_Time = _Now;
}


void HQ_AutoCar::imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{

   sensor_msgs::Imu temp_imu;
   temp_imu.header = imu_msg->header;
   temp_imu.orientation = imu_msg->orientation;
   temp_imu.orientation_covariance = imu_msg->orientation_covariance;
   temp_imu.angular_velocity.z = imu_msg->angular_velocity.z;
   temp_imu.angular_velocity_covariance = imu_msg->angular_velocity_covariance;
   // temp_imu.linear_acceleration = imu_msg->linear_acceleration;
   temp_imu.linear_acceleration.z = 9.8;
   temp_imu.linear_acceleration_covariance = imu_msg->linear_acceleration_covariance;

   temp_imu.header.stamp = ros::Time::now();
   //去除IUM零飘
   if( abs(loc_Car_linerV) < 10 && abs(loc_car_anglerV) < 0.01)
   {
   temp_imu.angular_velocity.z = loc_car_anglerV;
   }
   imu_pub_.publish(temp_imu);
}
/**************************************
Function: Publish odometer data
功能:发布里程计数据
***************************************/
void HQ_AutoCar::HQ_Publish_Odom()
{
   // 定义四元数的类，用于赋值，将角度转换为四元数，存储在类中
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);


   // 包获取机器人的速度。创建一个类
   nav_msgs::Odometry odom;
   odom.header.stamp = ros::Time::now();
   odom.header.frame_id = odom_frame_id_;

   // set the position
   odom.pose.pose.position.x = Robot_Pos.X; // 车辆的x坐标 
   odom.pose.pose.position.y = Robot_Pos.Y; // 车辆的y坐标
   odom.pose.pose.position.z = 0; // 车辆旋转的角度
   odom.pose.pose.orientation = odom_quat;  // 姿态，通过Z轴转角转换的四元数

   // set the velocity
   odom.child_frame_id = base_frame_id_;
   odom.twist.twist.linear.x = Robot_Vel.X;
   odom.twist.twist.linear.y = Robot_Vel.Y;
   odom.twist.twist.angular.z = Robot_Vel.Z; // 绕Z轴角速度

   // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
   // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
   if (Robot_Vel.X == 0)
      // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
          memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
   else
      // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
          memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
   // publish the message
   odom_pub_.publish(odom);
}


#if 0
void HQ_AutoCar::publishTFandOdom()
{
    ros::Time stamp = ros::Time::now();
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);


    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = stamp;
    odom_trans.header.frame_id = odom_frame_id_;
    odom_trans.child_frame_id = base_frame_id_;
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_.sendTransform(odom_trans);


    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_frame_id_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = v_lin_;
    odom.twist.twist.angular.z = v_ang_;
    odom_pub_.publish(odom);
}
#endif 

void HQ_AutoCar::Hq_auto_control()
{
    ros::Rate rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        if (dt > 0.0001)
        {
            //updateOdom(dt);
            //publishTFandOdom();
          //  HQ_Publish_Odom();
            last_time_ = current_time;
        }
        rate.sleep();
    }
}

HQ_AutoCar::~HQ_AutoCar() {}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_car_odom");
    HQ_AutoCar car;
    car.Hq_auto_control();
    return 0;
}