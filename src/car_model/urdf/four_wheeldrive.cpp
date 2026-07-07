#include"four_wheeldrive.h"
four_wheeldrive::four_wheeldrive(ros::NodeHandle& nh)
{
    
    nh.param("/car_param/one_wheell_v", wheeldrive.one_wheell_v, wheeldrive.one_wheell_v);
    nh.param("/car_param/one_wheell_angle", wheeldrive.one_wheell_angle, wheeldrive.one_wheell_angle);
    

    nh.param("/car_param/one_wheell_v", wheeldrive.one_wheell_v, wheeldrive.one_wheell_v);
    nh.param("/car_param/one_wheell_angle", wheeldrive.one_wheell_angle, wheeldrive.one_wheell_angle);
    nh.param("/car_param/two_wheell_v", wheeldrive.two_wheell_v, wheeldrive.two_wheell_v);
    nh.param("/car_param/two_wheell_angle", wheeldrive.two_wheell_angle, wheeldrive.two_wheell_angle);
    nh.param("/car_param/three_wheell_v", wheeldrive.three_wheell_v, wheeldrive.three_wheell_v);
    nh.param("/car_param/three_wheell_angle", wheeldrive.three_wheell_angle, wheeldrive.three_wheell_angle);
    nh.param("/car_param/four_wheell_v", wheeldrive.four_wheell_v, wheeldrive.four_wheell_v);
    nh.param("/car_param/four_wheell_angle", wheeldrive.four_wheell_angle, wheeldrive.four_wheell_angle);

    nh.param("/car_param/one_wheell_ateraldistance", wheeldrive.one_wheell_ateraldistance, wheeldrive.one_wheell_ateraldistance);
    nh.param("/car_param/one_wheell_Longitudinaldistance", wheeldrive.one_wheell_Longitudinaldistance, wheeldrive.one_wheell_Longitudinaldistance);
    nh.param("/car_param/two_wheell_ateraldistance", wheeldrive.two_wheell_ateraldistance, wheeldrive.two_wheell_ateraldistance);
    nh.param("/car_param/two_wheell_Longitudinaldistance", wheeldrive.two_wheell_Longitudinaldistance, wheeldrive.two_wheell_Longitudinaldistance);
    nh.param("/car_param/three_wheell_ateraldistance", wheeldrive.three_wheell_ateraldistance, wheeldrive.three_wheell_ateraldistance);
    nh.param("/car_param/three_wheell_Longitudinaldistance", wheeldrive.three_wheell_Longitudinaldistance, wheeldrive.three_wheell_Longitudinaldistance);
    nh.param("/car_param/four_wheell_ateraldistance", wheeldrive.four_wheell_ateraldistance, wheeldrive.four_wheell_ateraldistance);
    nh.param("/car_param/four_wheell_Longitudinaldistance", wheeldrive.four_wheell_Longitudinaldistance, wheeldrive.four_wheell_Longitudinaldistance);
    nh.param("/car_param/wheel_r", wheeldrive.wheel_r, wheeldrive.wheel_r);

    nh.param("/car_param/boby_w", wheeldrive.boby_w, wheeldrive.boby_w);
    nh.param("/car_param/boby_v", wheeldrive.boby_v, wheeldrive.boby_v);

    std::vector<double> position_vec(3);
    nh.param("/car_param/position", position_vec, position_vec);
    for (size_t i = 0; i < 3; ++i) 
    {
        car_odoem.position[i] = position_vec[i];
    }
    std::vector<double> velocity_vec(3);
    nh.param("/car_param/velocity", velocity_vec, velocity_vec);
    for (size_t i = 0; i < 3; ++i) 
    {
        car_odoem.velocity[i] = velocity_vec[i];
    }
    std::vector<double> orientation_vec(4);
    nh.param("/car_param/orientation", orientation_vec, orientation_vec);
    for (size_t i = 0; i < 4; ++i) 
    {
        car_odoem.orientation[i] = orientation_vec[i];
    }
    nh.param("/car_param/car_model", car_model, car_model);

}

int four_wheeldrive::speed_distribution(double v,double w)
{
    switch (car_model)
    {
        case 1:
            if (w==0) //直线和停下
            {
                wheeldrive.one_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.one_wheell_angle = 0;
                wheeldrive.two_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.two_wheell_angle = 0;
                wheeldrive.three_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.three_wheell_angle = 0;
                wheeldrive.four_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.four_wheell_angle = 0; 
            }
            else //圆弧和原地旋转
            {
                double R = v/w;
                wheeldrive.one_wheell_angle = atan(wheeldrive.one_wheell_Longitudinaldistance / (R - wheeldrive.one_wheell_ateraldistance));
                wheeldrive.one_wheell_v = w * (wheeldrive.one_wheell_Longitudinaldistance / sin(wheeldrive.one_wheell_angle))/wheeldrive.wheel_r;
                wheeldrive.two_wheell_angle = atan(wheeldrive.two_wheell_Longitudinaldistance / (R + wheeldrive.two_wheell_ateraldistance));
                wheeldrive.two_wheell_v = w * (wheeldrive.two_wheell_Longitudinaldistance / sin(wheeldrive.two_wheell_angle))/wheeldrive.wheel_r;
                wheeldrive.three_wheell_angle = -atan(wheeldrive.three_wheell_Longitudinaldistance / (R - wheeldrive.three_wheell_ateraldistance));
                wheeldrive.three_wheell_v = -w * (wheeldrive.three_wheell_Longitudinaldistance / sin(wheeldrive.three_wheell_angle))/wheeldrive.wheel_r;
                wheeldrive.four_wheell_angle = -atan(wheeldrive.four_wheell_Longitudinaldistance / (R + wheeldrive.four_wheell_ateraldistance));
                wheeldrive.four_wheell_v = -w * (wheeldrive.four_wheell_Longitudinaldistance / sin(wheeldrive.four_wheell_angle))/wheeldrive.wheel_r;
            }
            break;

        case 2:
            if(w==0)
            {
                wheeldrive.one_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.one_wheell_angle = 0;
                wheeldrive.two_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.two_wheell_angle = 0;
                wheeldrive.three_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.three_wheell_angle = 0;
                wheeldrive.four_wheell_v = v/wheeldrive.wheel_r;
                wheeldrive.four_wheell_angle = 0; 
            }
            else //圆弧和原地旋转
            {
                double R = v/w;
                wheeldrive.one_wheell_angle = atan((wheeldrive.one_wheell_Longitudinaldistance + wheeldrive.three_wheell_Longitudinaldistance)/(R - wheeldrive.one_wheell_ateraldistance));
                ROS_INFO("The answer is 1lun_jiaodu: %f", wheeldrive.one_wheell_angle * (180.0 / M_PI));
                wheeldrive.one_wheell_v =  w * ((wheeldrive.one_wheell_Longitudinaldistance + wheeldrive.three_wheell_Longitudinaldistance) / sin(wheeldrive.one_wheell_angle))/wheeldrive.wheel_r;
                ROS_INFO("The answer is 1lun_v: %f",wheeldrive.one_wheell_v * wheeldrive.wheel_r);
                wheeldrive.two_wheell_angle = atan((wheeldrive.two_wheell_Longitudinaldistance + wheeldrive.four_wheell_Longitudinaldistance) / ( R + wheeldrive.two_wheell_ateraldistance));
                ROS_INFO("The answer is 2lun_jiaodu: %f", wheeldrive.two_wheell_angle * (180.0 / M_PI));
                wheeldrive.two_wheell_v =  w * ((wheeldrive.two_wheell_Longitudinaldistance + wheeldrive.four_wheell_Longitudinaldistance) / sin(wheeldrive.two_wheell_angle))/wheeldrive.wheel_r;
                ROS_INFO("The answer is 2lun_v: %f",wheeldrive.two_wheell_v * wheeldrive.wheel_r);
                wheeldrive.three_wheell_angle = 0;
                wheeldrive.three_wheell_v = (R - wheeldrive.three_wheell_ateraldistance) * w /wheeldrive.wheel_r;
                ROS_INFO("The answer is 2lun_v: %f",wheeldrive.three_wheell_v * wheeldrive.wheel_r);
                wheeldrive.four_wheell_angle = 0;
                wheeldrive.four_wheell_v = (R + wheeldrive.four_wheell_ateraldistance) * w /wheeldrive.wheel_r;
                ROS_INFO("The answer is 2lun_v: %f",wheeldrive.four_wheell_v * wheeldrive.wheel_r);
                
            }
            
    }
    return 0;
}

int four_wheeldrive::speed_feedback(const sensor_msgs::JointState::ConstPtr& data)
{
     if (!data->effort.empty())
    {
        double front_right_wheel = data->velocity[3];
        double front_left_wheel = data->velocity[1];
        double rear_left_wheel = data->velocity[5];
        double rear_right_wheel = data->velocity[7];
        double front_left_steering = data->position[0];
        double front_right_steering = data->position[2];
        double rear_left_steering = data->position[4];
        double rear_right_steering = data->position[6];
        if (-0.00873 < front_left_steering && front_left_steering < 0.00873 &&
            -0.00873 < front_right_steering && front_right_steering < 0.00873 &&
            -0.00873 < rear_left_steering && rear_left_steering < 0.00873 &&
            -0.00873 < rear_right_steering && rear_right_steering < 0.00873) 
        {
            wheeldrive.boby_w = 0;
            wheeldrive.boby_v = (front_right_wheel+front_left_wheel+rear_left_wheel+rear_right_wheel) /4 * wheeldrive.wheel_r;
            ROS_INFO("feedback_l_v: %f",  wheeldrive.boby_v);
            ROS_INFO("feedback_l_w: %f",  wheeldrive.boby_w);
        }
        else //原地旋转和圆弧
        {
            if (-0.00873 < rear_left_steering && rear_left_steering < 0.00873 &&
            -0.00873 < rear_right_steering && rear_right_steering < 0.00873) //为阿克曼模型
            {
                double R_1 = (wheeldrive.one_wheell_Longitudinaldistance + wheeldrive.three_wheell_Longitudinaldistance) / tan(front_left_steering) + wheeldrive.one_wheell_ateraldistance;
                double R_2 = (wheeldrive.two_wheell_Longitudinaldistance + wheeldrive.four_wheell_Longitudinaldistance) / tan(front_right_steering) - wheeldrive.two_wheell_ateraldistance;
                double R = (R_1 + R_2) / 2;
                double w_3 = (rear_left_wheel * wheeldrive.wheel_r) / (R - wheeldrive.three_wheell_ateraldistance);
                double w_4 = (rear_right_wheel * wheeldrive.wheel_r) / (R + wheeldrive.four_wheell_ateraldistance);
                wheeldrive.boby_w = (w_3 + w_4)/2;
                wheeldrive.boby_v = wheeldrive.boby_w * R ;
                ROS_INFO("feedback_c_A_v: %f",  wheeldrive.boby_v);
                ROS_INFO("feedback_c_A_w: %f",  wheeldrive.boby_w);
            }
            else //4轮8驱模型
            {
                double lun1_R =  wheeldrive.one_wheell_Longitudinaldistance / sin(front_left_steering);
                double boby_w_1 = front_left_wheel * wheeldrive.wheel_r / lun1_R;
                double boby_v_1 = ((wheeldrive.one_wheell_Longitudinaldistance / tan(front_left_steering) + wheeldrive.one_wheell_ateraldistance) * boby_w_1);

                double lun2_R =  wheeldrive.two_wheell_Longitudinaldistance / sin(front_right_steering);
                double boby_w_2 = front_right_wheel * wheeldrive.wheel_r / lun2_R;
                double boby_v_2 = ((wheeldrive.two_wheell_Longitudinaldistance / tan(front_right_steering) - wheeldrive.two_wheell_ateraldistance) * boby_w_2);

                double lun3_R =  -wheeldrive.three_wheell_Longitudinaldistance / sin(rear_left_steering);
                double boby_w_3 = rear_left_wheel * wheeldrive.wheel_r / lun3_R;
                double boby_v_3 = ((-(wheeldrive.three_wheell_Longitudinaldistance / tan(rear_left_steering)) + wheeldrive.three_wheell_ateraldistance) * boby_w_3);

                double lun4_R =  -wheeldrive.four_wheell_Longitudinaldistance / sin(rear_right_steering);
                double boby_w_4 = rear_right_wheel * wheeldrive.wheel_r / lun4_R;
                double boby_v_4 = ((-(wheeldrive.four_wheell_Longitudinaldistance / tan(rear_right_steering)) - wheeldrive.four_wheell_ateraldistance) * boby_w_4);

                wheeldrive.boby_v = (boby_v_1 + boby_v_2 + boby_v_3 + boby_v_4)/4;
                wheeldrive.boby_w = (boby_w_1 + boby_w_2 + boby_w_3 + boby_w_4)/4;
                ROS_INFO("feedback_c_4_v: %f",  wheeldrive.boby_v);
                ROS_INFO("feedback_c_4_w: %f",  wheeldrive.boby_w);

            }
        }
    }
    return 0;
}

 nav_msgs::Odometry four_wheeldrive::calculate_odoem()
{
    tf::Quaternion q(car_odoem.orientation[0], car_odoem.orientation[1], car_odoem.orientation[2], car_odoem.orientation[3]);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    car_odoem.yaw = yaw;
    ROS_INFO("befor yaw: %f", car_odoem.yaw);
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    if ( wheeldrive.boby_w != 0 &&  wheeldrive.boby_v == 0)
    {    
        car_odoem.yaw +=  wheeldrive.boby_w * (1.0 / 10.0);
    } 
    else 
    {
        car_odoem.position[0] +=  wheeldrive.boby_v * cos(car_odoem.yaw) * (1.0 / 10.0);
        car_odoem.position[1] +=  wheeldrive.boby_v * sin(car_odoem.yaw) * (1.0 / 10.0);
        ROS_INFO("The answer is x: %f", car_odoem.position[0]);
        ROS_INFO("The answer is y: %f", car_odoem.position[1]);
        car_odoem.yaw +=  wheeldrive.boby_w * (1.0 / 10.0);
        ROS_INFO("The answer is JIAODU: %f", yaw * (180.0 / M_PI));
        ROS_INFO("The answer is JIAODU_time: %f",ros::Time::now().toSec() );
    }
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, car_odoem.yaw);
    car_odoem.orientation[0] = quaternion.x();
    car_odoem.orientation[1] = quaternion.y();
    car_odoem.orientation[2] = quaternion.z();
    car_odoem.orientation[3] = quaternion.w();

    geometry_msgs::Pose pose;
    pose.position.x = car_odoem.position[0];
    pose.position.y = car_odoem.position[1];
    pose.position.z = car_odoem.position[2];
    pose.orientation.x = car_odoem.orientation[0];
    pose.orientation.y = car_odoem.orientation[1];
    pose.orientation.z = car_odoem.orientation[2];
    pose.orientation.w = car_odoem.orientation[3];
    odom.pose.pose = pose;

    geometry_msgs::Twist twist;
    twist.linear.x = wheeldrive.boby_v;
    twist.linear.y = wheeldrive.boby_w;
    double boby_k = 0;
    if (wheeldrive.boby_v==0)
    {
        wheeldrive.boby_k = 10000;//原地旋转()
    }
    else
    {
        wheeldrive.boby_k = wheeldrive.boby_w/wheeldrive.boby_v;
    }
    twist.linear.z = wheeldrive.boby_k;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = wheeldrive.boby_w;
    odom.twist.twist = twist;
    return odom;
} 

  int four_wheeldrive::get_tf()
{
    std::shared_ptr<tf2_ros::TransformBroadcaster> br = std::make_shared<tf2_ros::TransformBroadcaster>();
    geometry_msgs::TransformStamped t;
    // 设置消息头
    t.header.stamp = ros::Time::now();
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    // 设置平移部分
    t.transform.translation.x = car_odoem.position[0];
    t.transform.translation.y = car_odoem.position[1];
    t.transform.translation.z = car_odoem.position[2];
    // 设置旋转部分
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, car_odoem.yaw);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    br->sendTransform(t);
    return 0;
}  

 int four_wheeldrive::msgs_too_old_wv()
 {
    wheeldrive.boby_w = 0;
    wheeldrive.boby_w = 0;
    return 0;
 }