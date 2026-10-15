#include "ros/ros.h"
#include "car_top/imu.h"//自定义消息类型

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"car_test");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<car_top::imu>("imu",10);

    //4.组织被发布的消息，编写发布逻辑并发布消息
/*#####################################
    uint16 accele_x_data
    uint16 accele_y_data
    uint16 accele_z_data
    uint16 gyros_x_data
    uint16 gyros_y_data
    uint16 gyros_z_data
####################################*/
    car_top::imu imu_test;
    imu_test.accele_x_data = 10000;
    imu_test.accele_y_data = 10000;
    imu_test.accele_z_data = 10000;
    imu_test.gyros_x_data = 10000;
    imu_test.gyros_y_data = 10000;
    imu_test.gyros_z_data = 10000;

    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(imu_test);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
