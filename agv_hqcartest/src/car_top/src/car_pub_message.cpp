#include "ros/ros.h"
#include "car_top/car_top.h"//自定义消息类型

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"car_test");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<car_top::car_top>("car_message",10);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    car_top::car_top car_top_test;
    car_top_test.back_wheel_speed = 10;
    car_top_test.battery_level = 20;
    car_top_test.error_flag = 0x00;
    car_top_test.turn_angle = 0.4;
    car_top_test.left_speed = 12.1313;
    car_top_test.right_speed = 12.3131;

    ros::Rate r(1);
    while (ros::ok())
    {
        pub.publish(car_top_test);
        r.sleep();
        ros::spinOnce();
    }
    return 0;
}
