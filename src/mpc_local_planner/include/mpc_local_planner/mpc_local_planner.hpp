#ifndef MPC_LOCAL_PLANNER_HPP
#define MPC_LOCAL_PLANNER_HPP

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/buffer.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include "mpc_local_planner/nmpc_core/diffmpc.hpp"

namespace mpc_local_planner
{

class MpcLocalPlanner : public nav_core::BaseLocalPlanner
{
public:
    enum class State { ALIGN_STEERING, ALIGNING, RECENTERING, TRACKING };
    MpcLocalPlanner() = default;
    ~MpcLocalPlanner() override = default;

    void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
    bool isGoalReached() override;

private:
    void loadParameters(const ros::NodeHandle& private_nh);
    void initializeRosInterfaces(ros::NodeHandle& nh);
    void initializeMpc();

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    double getYaw(const geometry_msgs::Quaternion& q) const;
    bool getRobotPose(Eigen::Vector3d& state) const;
    bool transformPoseToPlanFrame(const geometry_msgs::PoseStamped& source_pose,
                                  geometry_msgs::PoseStamped& plan_pose) const;
    bool normalizePlanFrames(const std::vector<geometry_msgs::PoseStamped>& plan,
                             const std::string& plan_frame,
                             std::vector<geometry_msgs::PoseStamped>& normalized_plan) const;

    void convertPlanToReference(const std::vector<geometry_msgs::PoseStamped>& plan);
    void setReferencePositions(const std::vector<geometry_msgs::PoseStamped>& plan);
    void smoothReferencePath(int window);
    void updateReferenceYaw();
    void updateReferenceCurvature(int smoothing_window);
    void updateSpeedProfile(int smoothing_window);
    void applyStopProfile(double stop_distance);
    std::vector<double> smoothVector(const std::vector<double>& data, int window) const;

    std::tuple<int, double> calcForwardNearestIndex(double current_x, double current_y);
    void updateStateForNewPlan();
    bool prepareAlignmentSteering(const Eigen::Vector3d& current_state, geometry_msgs::Twist& cmd_vel);
    bool handleAlignment(const Eigen::Vector3d& current_state, geometry_msgs::Twist& cmd_vel);
    Eigen::VectorXd makeInPlaceRotationCommand(double omega) const;
    bool stepSteeringToward(const Eigen::Vector4d& target, Eigen::Vector4d& stepped_steer) const;
    void resetControlToReference(int min_index, double min_e);
    bool solveMpcCommand(const Eigen::Vector3d& state, int min_index, Eigen::VectorXd& U_solve);

    bool publishWheelCommands(Eigen::VectorXd& U);
    void publishZeroCommands() const;
    void rememberSteeringCommand(const Eigen::VectorXd& U) const;
    void computeEquivalentTwist(const Eigen::VectorXd& U, geometry_msgs::Twist& cmd_vel);
    bool recenterSteering(geometry_msgs::Twist& cmd_vel);
    bool checkGoalReached(const Eigen::Vector3d& current_state) const;

    bool initialized_ = false;

    // Alignment state
    State state_ = State::TRACKING;
    double align_yaw_threshold_ = M_PI / 3.0;  // 60 deg    进入原地旋转的阈值
    double align_yaw_exit_threshold_ = M_PI / 18.0;  // 10 deg 退出原地旋转的阈值
    double align_recenter_max_step_ = 0.05;    // rad per control cycle
    double align_recenter_tolerance_ = 0.01;   // rad
    Eigen::Vector4d align_recenter_steer_ = Eigen::Vector4d::Zero();
    mutable Eigen::Vector4d last_steer_cmd_ = Eigen::Vector4d::Zero();
    mutable bool has_last_steer_cmd_ = false;
    mutable Eigen::VectorXd last_control_command_;
    mutable ros::WallTime last_control_command_time_;
    mutable bool has_last_control_command_ = false;
    double align_max_omega_ = 0.5;             // rad/s
    double align_kp_ = 1.0;
    double L_front_ = 0.4615;
    double L_rear_ = 0.4725;
    tf2_ros::Buffer* tf_ = nullptr;
    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;

    // MPC
    parameters param_;
    std::unique_ptr<diffMpcController> mpc_;

    // Reference trajectory
    std::vector<double> r_x_, r_y_, ryaw_, rcurvature_, speed_profile_;
    std::string plan_frame_;
    bool has_plan_ = false;
    int last_min_index_ = 0;

    // Odometry
    ros::Subscriber odom_sub_;
    geometry_msgs::PoseStamped latest_odom_pose_;
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

    // Parameters
    double wheel_radius_ = 0.15;
    double max_speed_ = 1.5;
    double target_speed_ = 1.0;
    double transform_timeout_ = 0.05;
    int forward_window_ = 80;
    int back_buffer_ = 10;
    double goal_xy_tolerance_ = 0.1;
    double goal_yaw_tolerance_ = 0.2;
};

} // namespace mpc_local_planner

#endif // MPC_LOCAL_PLANNER_HPP
