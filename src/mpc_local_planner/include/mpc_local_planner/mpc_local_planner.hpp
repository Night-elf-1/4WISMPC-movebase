#ifndef MPC_LOCAL_PLANNER_HPP
#define MPC_LOCAL_PLANNER_HPP

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <mutex>
#include <vector>

#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

namespace mpc_local_planner
{

class MpcLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
    enum class State { ALIGNING, RECENTERING, TRACKING };
    MpcLocalPlanner();
    ~MpcLocalPlanner();

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double getYaw(const geometry_msgs::Quaternion& q) const;
    bool getRobotPose(Eigen::Vector3d& state) const;
    void convertPlanToReference(const std::vector<geometry_msgs::PoseStamped>& plan);
    std::vector<double> smoothVector(const std::vector<double>& data, int window) const;
    std::tuple<int, double> calcForwardNearestIndex(double current_x, double current_y);
    void publishWheelCommands(const Eigen::VectorXd& U);
    void publishZeroCommands() const;
    void computeEquivalentTwist(const Eigen::VectorXd& U, geometry_msgs::Twist& cmd_vel);
    bool recenterSteering(geometry_msgs::Twist& cmd_vel);
    bool checkGoalReached(const Eigen::Vector3d& current_state,
                          double& dist_to_goal, double& dyaw) const;

    bool initialized_ = false;
    std::string name_;

    // Alignment state
    State state_ = State::TRACKING;
    double align_yaw_threshold_ = M_PI / 3.0;  // 60 deg    进入原地旋转的阈值
    double align_yaw_exit_threshold_ = M_PI / 18.0;  // 10 deg 退出原地旋转的阈值
    double align_recenter_max_step_ = 0.05;    // rad per control cycle
    double align_recenter_tolerance_ = 0.01;   // rad
    Eigen::Vector4d align_recenter_steer_ = Eigen::Vector4d::Zero();
    double align_max_omega_ = 0.5;             // rad/s
    double align_kp_ = 1.0;
    double L_front_ = 0.4615;
    double L_rear_ = 0.4725;
    tf2_ros::Buffer* tf_ = nullptr;
    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

    // MPC
    parameters param_;
    std::unique_ptr<diffMpcController> mpc_;
    std::unique_ptr<KinematicModel_MPC> agv_model_;

    // Reference trajectory
    std::vector<double> r_x_, r_y_, ryaw_, rcurvature_, speed_profile_;
    bool has_plan_ = false;
    int last_min_index_ = 0;

    // Odometry
    ros::Subscriber odom_sub_;
    nav_msgs::Odometry latest_odom_;
    bool has_odom_ = false;
    mutable std::mutex mutex_;

    // Wheel command publishers
    ros::Publisher pub_whell_front_L_;
    ros::Publisher pub_steer_front_L_;
    ros::Publisher pub_whell_front_R_;
    ros::Publisher pub_steer_front_R_;
    ros::Publisher pub_whell_rear_L_;
    ros::Publisher pub_steer_rear_L_;
    ros::Publisher pub_whell_rear_R_;
    ros::Publisher pub_steer_rear_R_;
    ros::WallTime last_wheel_command_publish_time_;
    ros::WallTime wheel_command_rate_window_start_;
    int wheel_command_publish_count_ = 0;

    // Parameters
    double wheel_radius_ = 0.15;
    double max_speed_ = 1.5;
    double target_speed_ = 1.0;
    int forward_window_ = 80;
    int back_buffer_ = 10;
    double goal_xy_tolerance_ = 0.1;
    double goal_yaw_tolerance_ = 0.2;
};

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_HPP
