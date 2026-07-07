#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>
#include <chrono>
#include "nmpc_4wisnew2/diffmpc.hpp"
#include <vector>
#include <mutex>
#include <tuple>
#include <cmath>

using namespace std;

class Nmpc4wisnew2ControllerNode
{
public:
    Nmpc4wisnew2ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : mpc_(param_.NX, param_.NU, param_.NP, param_.NC),
          agv_model_(0.0, 0.0, 0.0, 0.0, param_.L, param_.W, param_.dt),
          tf_buffer_(ros::Duration(30.0)),
          tf_listener_(tf_buffer_)
    {
        // 读取参数
        pnh.param<double>("wheel_radius", wheel_radius_, 0.15);
        pnh.param<double>("control_rate", control_rate_, 10.0);
        pnh.param<bool>("use_tf_for_state", use_tf_for_state_, true);
        pnh.param<double>("spawn_x", spawn_x_, 10.0);
        pnh.param<double>("spawn_y", spawn_y_, 0.0);
        pnh.param<int>("forward_window", forward_window_, 80);
        pnh.param<int>("back_buffer", back_buffer_, 10);

        // 订阅参考路径、曲率、速度和里程计
        path_sub_ = nh.subscribe("/reference_path", 1, &Nmpc4wisnew2ControllerNode::pathCallback, this);
        curvature_sub_ = nh.subscribe("/reference_curvature", 1, &Nmpc4wisnew2ControllerNode::curvatureCallback, this);
        speed_sub_ = nh.subscribe("/reference_speeds", 1, &Nmpc4wisnew2ControllerNode::speedCallback, this);
        odom_sub_ = nh.subscribe("/odom", 1, &Nmpc4wisnew2ControllerNode::odomCallback, this);

        // 发布 8 个控制指令
        pub_whell_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_svelocity_controller/command", 1);
        pub_steer_front_L_ = nh.advertise<std_msgs::Float64>("/smart/front_left_str_controller/command", 1);
        pub_whell_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_velocity_controller/command", 1);
        pub_steer_front_R_ = nh.advertise<std_msgs::Float64>("/smart/front_right_str_controller/command", 1);
        pub_whell_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_velocity_controller/command", 1);
        pub_steer_rear_L_  = nh.advertise<std_msgs::Float64>("/smart/rear_left_str_controller/command", 1);
        pub_whell_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_velocity_controller/command", 1);
        pub_steer_rear_R_  = nh.advertise<std_msgs::Float64>("/smart/rear_right_str_controller/command", 1);

        // 调试：发布最近参考点
        nearest_point_pub_ = nh.advertise<visualization_msgs::Marker>("/nmpc_4wisnew2_nearest_point", 1);

        // 控制定时器
        timer_ = nh.createTimer(ros::Duration(1.0 / control_rate_), &Nmpc4wisnew2ControllerNode::controlLoop, this);

        ROS_INFO("NMPC 4WIS new2 controller initialized. wheel_radius=%.3f, control_rate=%.1f Hz",
                 wheel_radius_, control_rate_);
    }

private:
    void pathCallback(const nav_msgs::Path::ConstPtr& msg)
    {
        lock_guard<mutex> lock(mutex_);
        r_x_.clear();
        r_y_.clear();
        ryaw_.clear();
        for(const auto& pose : msg->poses)
        {
            r_x_.push_back(pose.pose.position.x);
            r_y_.push_back(pose.pose.position.y);
            ryaw_.push_back(getYaw(pose.pose.orientation));
        }
        mpc_.smooth_yaw(ryaw_);
        has_path_ = true;
        ROS_INFO_ONCE("Received reference path with %zu points.", r_x_.size());
    }

    void curvatureCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        lock_guard<mutex> lock(mutex_);
        rcurvature_ = msg->data;
        has_curvature_ = true;
    }

    void speedCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    {
        lock_guard<mutex> lock(mutex_);
        speed_profile_ = msg->data;
        has_speed_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        lock_guard<mutex> lock(mutex_);
        latest_odom_ = *msg;
        has_odom_ = true;
    }

    double getYaw(const geometry_msgs::Quaternion& q)
    {
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(quat);
        double r, p, y;
        m.getRPY(r, p, y);
        return y;
    }

    bool getCurrentState(Eigen::Vector3d& state)
    {
        if(use_tf_for_state_)
        {
            // 通过 TF map -> base_link 获取小车在 map 下的位姿
            geometry_msgs::TransformStamped transform;
            try
            {
                transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
            }
            catch(tf2::TransformException& ex)
            {
                ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", ex.what());
                // 失败时回退到里程计 + spawn 偏移
                if(!has_odom_) return false;
                state << latest_odom_.pose.pose.position.x + spawn_x_,
                         latest_odom_.pose.pose.position.y + spawn_y_,
                         getYaw(latest_odom_.pose.pose.orientation);
                return true;
            }
            state << transform.transform.translation.x,
                     transform.transform.translation.y,
                     getYaw(transform.transform.rotation);
            return true;
        }
        else
        {
            if(!has_odom_) return false;
            state << latest_odom_.pose.pose.position.x + spawn_x_,
                     latest_odom_.pose.pose.position.y + spawn_y_,
                     getYaw(latest_odom_.pose.pose.orientation);
            return true;
        }
    }

    void controlLoop(const ros::TimerEvent&)
    {
        lock_guard<mutex> lock(mutex_);

        if(!has_path_ || !has_curvature_ || !has_speed_ || !has_odom_)
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for reference path/curvature/speeds/odometry...");
            return;
        }

        // 数据长度校验
        if(rcurvature_.size() != r_x_.size() || speed_profile_.size() != r_x_.size())
        {
            ROS_WARN_THROTTLE(5.0, "Reference data size mismatch: path=%zu, curvature=%zu, speeds=%zu",
                              r_x_.size(), rcurvature_.size(), speed_profile_.size());
            return;
        }

        // 获取当前状态
        Eigen::Vector3d initial_x;
        if(!getCurrentState(initial_x))
        {
            ROS_WARN_THROTTLE(5.0, "Failed to get current state.");
            return;
        }

        // 查找最近点（只在前向窗口内搜索，避免回退并减少计算量）
        auto [min_index, min_e] = calcForwardNearestIndex(initial_x(0), initial_x(1));

        // 发布最近参考点用于调试
        publishNearestPoint(min_index);

        // 如果横向误差过大或卡住，重置 MPC 内部控制量为参考控制量，避免 U 累积发散
        if(min_e > 1.0)
        {
            resetControlToReference(min_index);
            ROS_WARN_THROTTLE(1.0, "Lateral error %.3f, resetting NMPC control to reference.", min_e);
        }

        // 到达终点附近，停止
        if(min_index >= static_cast<int>(r_x_.size()) - 15)
        {
            ROS_INFO("Reached end of reference path. Stopping.");
            publishZeroCommands();
            timer_.stop();
            return;
        }

        // 更新运动学模型状态（NMPC 需要当前纵向速度 v 来构建预测时域）
        agv_model_.x   = initial_x(0);
        agv_model_.y   = initial_x(1);
        agv_model_.yaw = initial_x(2);
        agv_model_.v   = speed_profile_[min_index];

        // NMPC 求解，统计耗时
        Eigen::VectorXd U_solve;
        double solve_time_ms = 0.0;
        try
        {
            auto t_start = std::chrono::high_resolution_clock::now();
            U_solve = mpc_.mpc_solve(r_x_, r_y_, ryaw_, rcurvature_, speed_profile_, initial_x, min_index, min_e, agv_model_, param_);
            auto t_end = std::chrono::high_resolution_clock::now();
            solve_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        }
        catch(const std::exception& e)
        {
            ROS_ERROR("NMPC solve failed: %s", e.what());
            publishZeroCommands();
            return;
        }

        // 发布控制量
        // U_solve(0..3): 四个轮子线速度 -> 除以 wheel_radius 得到角速度
        // U_solve(4..7): 四个轮子转角
        std_msgs::Float64 msg;

        msg.data = U_solve(0) / wheel_radius_;
        pub_whell_front_L_.publish(msg);

        msg.data = U_solve(1) / wheel_radius_;
        pub_whell_rear_L_.publish(msg);

        msg.data = U_solve(2) / wheel_radius_;
        pub_whell_rear_R_.publish(msg);

        msg.data = U_solve(3) / wheel_radius_;
        pub_whell_front_R_.publish(msg);

        msg.data = U_solve(4);
        pub_steer_front_L_.publish(msg);

        msg.data = U_solve(5);
        pub_steer_rear_L_.publish(msg);

        msg.data = U_solve(6);
        pub_steer_rear_R_.publish(msg);

        msg.data = U_solve(7);
        pub_steer_front_R_.publish(msg);

        ROS_INFO_THROTTLE(1.0, "NMPC(4WISnew2): nearest_idx=%d, nearest_dist=%.3f, solve_time=%.2fms, v=[%.2f, %.2f, %.2f, %.2f]",
                          min_index, min_e, solve_time_ms,
                          U_solve(0), U_solve(1), U_solve(2), U_solve(3));
        ROS_INFO_THROTTLE(1.0, "NMPC(4WISnew2): steering=[%.3f, %.3f, %.3f, %.3f]",
                          U_solve(4), U_solve(5), U_solve(6), U_solve(7));
    }

    void publishZeroCommands()
    {
        std_msgs::Float64 msg;
        msg.data = 0.0;
        pub_whell_front_L_.publish(msg);
        pub_steer_front_L_.publish(msg);
        pub_whell_front_R_.publish(msg);
        pub_steer_front_R_.publish(msg);
        pub_whell_rear_L_.publish(msg);
        pub_steer_rear_L_.publish(msg);
        pub_whell_rear_R_.publish(msg);
        pub_steer_rear_R_.publish(msg);
    }

    std::tuple<int, double> calcForwardNearestIndex(double current_x, double current_y)
    {
        int n = r_x_.size();
        int start_idx = std::max(0, last_min_index_ - back_buffer_);
        int end_idx = std::min(n, last_min_index_ + forward_window_);

        double mind = std::numeric_limits<double>::max();
        int min_index = start_idx;

        for(int i = start_idx; i < end_idx; i++)
        {
            double idx = current_x - r_x_[i];
            double idy = current_y - r_y_[i];
            double d_e = std::sqrt(idx*idx + idy*idy);   // 欧氏距离

            if(d_e < mind)
            {
                mind = d_e;
                min_index = i;
            }
        }

        // 如果前向窗口内找不到更合适的点（偏差极大），回退到全局搜索
        if(mind > 5.0 && start_idx > 0)
        {
            return mpc_.calc_ref_trajectory(current_x, current_y, r_x_, r_y_, ryaw_);
        }

        last_min_index_ = min_index;
        return std::make_tuple(min_index, mind);
    }

    void publishNearestPoint(int min_index)
    {
        if(nearest_point_pub_.getNumSubscribers() == 0) return;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "nmpc_4wisnew2_nearest";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = r_x_[min_index];
        marker.pose.position.y = r_y_[min_index];
        marker.pose.position.z = 0.5;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        marker.lifetime = ros::Duration(0.2);
        nearest_point_pub_.publish(marker);
    }

    void resetControlToReference(int min_index)
    {
        double v_r = speed_profile_[min_index];
        double k_r = rcurvature_[min_index];
        double omega_r = v_r * k_r;
        double L = param_.L;
        double W = param_.W;
        double xw[4] = { L, -L, -L, L };
        double yw[4] = { W,  W, -W, -W };

        for(int i = 0; i < 4; i++)
        {
            double vx_i = v_r - yw[i] * omega_r;
            double vy_i = xw[i] * omega_r;
            mpc_.U(i) = std::sqrt(vx_i * vx_i + vy_i * vy_i) * (vx_i >= 0 ? 1.0 : -1.0);
            mpc_.U(i + 4) = std::atan2(vy_i, vx_i);
        }
    }

private:
    parameters param_;
    diffMpcController mpc_;
    KinematicModel_MPC agv_model_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber path_sub_;
    ros::Subscriber curvature_sub_;
    ros::Subscriber speed_sub_;
    ros::Subscriber odom_sub_;

    ros::Publisher pub_whell_front_L_;
    ros::Publisher pub_steer_front_L_;
    ros::Publisher pub_whell_front_R_;
    ros::Publisher pub_steer_front_R_;
    ros::Publisher pub_whell_rear_L_;
    ros::Publisher pub_steer_rear_L_;
    ros::Publisher pub_whell_rear_R_;
    ros::Publisher pub_steer_rear_R_;

    ros::Timer timer_;
    ros::Publisher nearest_point_pub_;

    mutex mutex_;
    vector<double> r_x_, r_y_, ryaw_, rcurvature_, speed_profile_;
    nav_msgs::Odometry latest_odom_;
    bool has_path_ = false;
    bool has_curvature_ = false;
    bool has_speed_ = false;
    bool has_odom_ = false;

    double wheel_radius_ = 0.15;
    double control_rate_ = 10.0;
    bool use_tf_for_state_ = true;
    double spawn_x_ = 10.0;
    double spawn_y_ = 0.0;

    int forward_window_ = 80;
    int back_buffer_ = 10;
    int last_min_index_ = 0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "nmpc_4wisnew2_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    Nmpc4wisnew2ControllerNode controller(nh, pnh);

    ros::spin();

    return 0;
}
