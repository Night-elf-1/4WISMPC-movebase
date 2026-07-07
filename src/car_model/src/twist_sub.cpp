#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include"four_wheeldrive.h"
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

ros::Publisher pub_allocation_speed;
ros::Publisher pub_speed_feedback;
// ros::Publisher pub_odom;
ros::Time lastMsg_time ; 
bool publish_odom_tf = true;

ros::Publisher pub_steer_front_L;
ros::Publisher pub_whell_front_L;

ros::Publisher pub_steer_front_R;
ros::Publisher pub_whell_front_R;

ros::Publisher pub_steer_rear_L;
ros::Publisher pub_whell_rear_L;

ros::Publisher pub_steer_rear_R;
ros::Publisher pub_whell_rear_R;

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg,four_wheeldrive*four_wd)
{
    lastMsg_time = ros::Time::now();
    std_msgs::Float64MultiArray processed_msg;
    //开始分配
    four_wd->speed_distribution( msg->linear.x,msg->angular.z);
    ROS_INFO("get w and v:  %f %f",msg->angular.z,msg->linear.x);
    const auto& whell_data  = four_wd->getPrivateData();
    processed_msg.data.resize(8); // 我们只处理8个数字，所以只发布8个数字
    processed_msg.data[0] =  whell_data.one_wheell_v; 
    processed_msg.data[1] =  whell_data.one_wheell_angle;
    processed_msg.data[2] =  whell_data.two_wheell_v;
    processed_msg.data[3] =  whell_data.two_wheell_angle;
    processed_msg.data[4] =  whell_data.three_wheell_v;
    processed_msg.data[5] =  whell_data.three_wheell_angle;
    processed_msg.data[6] =  whell_data.four_wheell_v;
    processed_msg.data[7] =  whell_data.four_wheell_angle;
    ROS_INFO("whell4 linear jiaod:  %f ",whell_data.four_wheell_angle);
    ROS_INFO("whell4 linear velocity:  %f ",whell_data.four_wheell_v);
    //发布每个轮子的速度和转角
    pub_allocation_speed.publish(processed_msg);

    //仿真测试
    std_msgs::Float64 msgfrontL_steer;
    std_msgs::Float64 msgfrontL_v;
    msgfrontL_steer.data = whell_data.one_wheell_angle;
    msgfrontL_v.data = whell_data.one_wheell_v;

    std_msgs::Float64 msgfrontR_steer;
    std_msgs::Float64 msgfrontR_v;
    msgfrontR_steer.data = whell_data.two_wheell_angle;
    msgfrontR_v.data = whell_data.two_wheell_v;

    std_msgs::Float64 msgrearL_steer;
    std_msgs::Float64 msgrearL_v;
    msgrearL_steer.data = whell_data.three_wheell_angle;
    msgrearL_v.data = whell_data.three_wheell_v;
 
    std_msgs::Float64 msgrearR_steer;
    std_msgs::Float64 msgrearR_v;
    // msgrearR_steer.data =  whell_data.four_wheell_angle;
    // msgrearR_v.data = whell_data.four_wheell_v;
    // pub_steer_front_L.publish(msgfrontL_steer);
    // pub_whell_front_L.publish(msgfrontL_v);
    // pub_steer_front_R.publish(msgfrontR_steer);
    // pub_whell_front_R.publish(msgfrontR_v);
    // pub_steer_rear_L.publish(msgrearL_steer);
    // pub_whell_rear_L.publish(msgrearL_v);
    // pub_steer_rear_R.publish(msgrearR_steer);
    // pub_whell_rear_R.publish(msgrearR_v);
}

void twistCallback_k(const geometry_msgs::Twist::ConstPtr& msg,four_wheeldrive*four_wd)
{
    lastMsg_time = ros::Time::now();
    std_msgs::Float64MultiArray processed_msg;
    double self_v = msg->linear.x;
    double self_w = msg->linear.x*msg->angular.z;
    if (msg->angular.z>10)
    {
        self_v = 0;
        self_w = 0.2;
    }
    four_wd->speed_distribution(self_v,self_w);
    ROS_INFO("get w_A and v_A:  %f %f",msg->angular.z,msg->linear.x);
    const auto& whell_data  = four_wd->getPrivateData();
    processed_msg.data.resize(8); // 我们只处理8个数字，所以只发布8个数字
    processed_msg.data[0] =  whell_data.one_wheell_v; 
    processed_msg.data[1] =  whell_data.one_wheell_angle;
    processed_msg.data[2] =  whell_data.two_wheell_v;
    processed_msg.data[3] =  whell_data.two_wheell_angle;
    processed_msg.data[4] =  whell_data.three_wheell_v;
    processed_msg.data[5] =  whell_data.three_wheell_angle;
    processed_msg.data[6] =  whell_data.four_wheell_v;
    processed_msg.data[7] =  whell_data.four_wheell_angle;
    ROS_INFO("whell4 linear jiaod:  %f ",whell_data.four_wheell_angle);
    ROS_INFO("whell4 linear velocity:  %f ",whell_data.four_wheell_v);
    //发布每个轮子的速度和转角
    pub_allocation_speed.publish(processed_msg);

    //仿真测试
    std_msgs::Float64 msgfrontL_steer;
    std_msgs::Float64 msgfrontL_v;
    msgfrontL_steer.data = whell_data.one_wheell_angle;
    msgfrontL_v.data = whell_data.one_wheell_v;

    std_msgs::Float64 msgfrontR_steer;
    std_msgs::Float64 msgfrontR_v;
    msgfrontR_steer.data = whell_data.two_wheell_angle;
    msgfrontR_v.data = whell_data.two_wheell_v;

    std_msgs::Float64 msgrearL_steer;
    std_msgs::Float64 msgrearL_v;
    msgrearL_steer.data = whell_data.three_wheell_angle;
    msgrearL_v.data = whell_data.three_wheell_v;
 
    std_msgs::Float64 msgrearR_steer;
    std_msgs::Float64 msgrearR_v;
    // msgrearR_steer.data =  whell_data.four_wheell_angle;
    // msgrearR_v.data = whell_data.four_wheell_v;
    // pub_steer_front_L.publish(msgfrontL_steer);
    // pub_whell_front_L.publish(msgfrontL_v);
    // pub_steer_front_R.publish(msgfrontR_steer);
    // pub_whell_front_R.publish(msgfrontR_v);
    // pub_steer_rear_L.publish(msgrearL_steer);
    // pub_whell_rear_L.publish(msgrearL_v);
    // pub_steer_rear_R.publish(msgrearR_steer);
    // pub_whell_rear_R.publish(msgrearR_v);
}

void callbackJointStates(const sensor_msgs::JointState::ConstPtr& data,four_wheeldrive*four_wd)
{
    geometry_msgs::Twist self_twist;
    four_wd->speed_feedback(data);
    const auto& whell_data  = four_wd->getPrivateData();
    self_twist.linear.x = whell_data.boby_v;
    self_twist.angular.z = whell_data.boby_v;
    //发布反馈
    pub_speed_feedback.publish(self_twist);   
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_subscriber");
    ros::NodeHandle n;
    n.param("publish_odom_tf", publish_odom_tf, true);
    four_wheeldrive four_WD(n);
    

    pub_allocation_speed = n.advertise<std_msgs::Float64MultiArray>("processed_data", 10);
    pub_speed_feedback = n.advertise<geometry_msgs::Twist>("cmd_vel_speed_feedback", 10);
    // if (publish_odom_tf)
    // {
    //     pub_odom = n.advertise<nav_msgs::Odometry>("odom", 50);
    // }

    ros::Subscriber sub_v_w = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, boost::bind(&twistCallback,_1,&four_WD));
    ros::Subscriber sub_v_k = n.subscribe<geometry_msgs::Twist>("cmd_vel_k", 10, boost::bind(&twistCallback_k,_1,&four_WD));
    ros::Subscriber joint_states_sub_ = n.subscribe<sensor_msgs::JointState>("/smart/joint_states", 10,boost::bind(&callbackJointStates,_1,&four_WD));
   
    pub_whell_front_L = n.advertise<std_msgs::Float64>("/smart/front_left_svelocity_controller/command", 1);
    pub_steer_front_L = n.advertise<std_msgs::Float64>("/smart/front_left_str_controller/command", 1);
    pub_whell_front_R = n.advertise<std_msgs::Float64>("/smart/front_right_velocity_controller/command", 1);
    pub_steer_front_R = n.advertise<std_msgs::Float64>("/smart/front_right_str_controller/command", 1);
    pub_whell_rear_L = n.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 1);
    pub_steer_rear_L = n.advertise<std_msgs::Float64>("/smart/rear_left_str_controller/command", 1);
    pub_whell_rear_R = n.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 1);
    pub_steer_rear_R = n.advertise<std_msgs::Float64>("/smart/rear_right_str_controller/command", 1);
    
    ros::Rate loop_rate(10);
    ros::Duration timeout(0.2);
    while (ros::ok())
    {
        // 使用官方/ground truth 里程计时，在 launch 中关闭自定义里程计发布
        // if (publish_odom_tf)
        // {
        //     const auto& odom_msg = four_WD.calculate_odoem();
        //     // ROS_INFO("The answer is time: %f",ros::Time::now().toSec() );
        //     pub_odom.publish(odom_msg);
        //     four_WD.get_tf();
        // }
        // 判断每给信号将停止
        ros::Duration delta_last_msg_time = ros::Time::now() - lastMsg_time;
        bool msgs_too_old = delta_last_msg_time > timeout;
        if (msgs_too_old)
        {
            four_WD.msgs_too_old_wv();
            std_msgs::Float64 too_old_msg ;
            too_old_msg.data = 0;
            pub_steer_front_L.publish(too_old_msg);
            pub_whell_front_L.publish(too_old_msg);
            pub_steer_front_R.publish(too_old_msg);
            pub_whell_front_R.publish(too_old_msg);
            pub_steer_rear_L.publish(too_old_msg);
            pub_whell_rear_L.publish(too_old_msg);
            pub_steer_rear_R.publish(too_old_msg);
            pub_whell_rear_R.publish(too_old_msg);
        }
        ros::spinOnce(); 
        loop_rate.sleep();
    }
    return 0;
}