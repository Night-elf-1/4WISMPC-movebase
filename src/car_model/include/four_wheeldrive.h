#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <yaml-cpp/yaml.h>


struct wheel
{
    //分配的速度和转角
    double one_wheell_v;
    double one_wheell_angle;
    double two_wheell_v;
    double two_wheell_angle;
    double three_wheell_v;
    double three_wheell_angle;
    double four_wheell_v;
    double four_wheell_angle;

    //车的尺寸
    double one_wheell_ateraldistance;
    double one_wheell_Longitudinaldistance;
    double two_wheell_ateraldistance;
    double two_wheell_Longitudinaldistance;
    double three_wheell_ateraldistance;
    double three_wheell_Longitudinaldistance ;
    double four_wheell_ateraldistance;
    double four_wheell_Longitudinaldistance ;
    double wheel_r ;

    //反馈速度和角速度
    double boby_w ;
    double boby_v ;
    double boby_k ; 
};

struct odoem_inf
{
    double yaw;
    double position[3]  ;
    double velocity[3] ;  // 假设线速度为0.1 m/s
    double orientation[4] ;  // 初始姿态为无旋转
};


class four_wheeldrive
{
    private: 
        wheel wheeldrive;
    //模式选择 1为4轮8驱，2为阿克曼
        int car_model ;
    // 里程计信息
        odoem_inf car_odoem;
    //
    std::shared_ptr<tf2_ros::TransformBroadcaster> br = std::make_shared<tf2_ros::TransformBroadcaster>();
    public:
        four_wheeldrive(ros::NodeHandle& nh); //初始化加载yaml参数
        int speed_distribution(double v,double w); //进行速度分配，并把分配好的速度和角度赋值给私有成员wheel
        int speed_feedback(const sensor_msgs::JointState::ConstPtr& data); //函数输入4个轮子速度和角度反解机器人的w和v
        nav_msgs::Odometry calculate_odoem(); //计算里程计信息
        int get_tf(); ////计算tf信息
        int msgs_too_old_wv(); //没有下发速度和角速度，让机器人的速度和角速度为0
        const wheel& getPrivateData() const {
        return wheeldrive;
    }; //在私有成员里面获取分配好的速度，和反解后的速度和角速度.
        
};