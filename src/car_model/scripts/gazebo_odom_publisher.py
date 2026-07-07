#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
订阅 Gazebo 的 /gazebo/model_states，发布 ground truth 里程计。
发布内容：
  - /odom 话题 (nav_msgs/Odometry)
  - odom -> base_link 的 TF
"""

import math
import rospy
import tf
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped
import tf2_ros


class GazeboOdomPublisher:
    def __init__(self):
        rospy.init_node("gazebo_odom_publisher")

        self.model_name = rospy.get_param("~model_name", "sevnce_robot")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.publish_rate = rospy.get_param("~publish_rate", 50.0)

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=50)
        self.br = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb)

        self.current_pose = None
        self.current_twist = None

        rospy.loginfo(
            "GazeboOdomPublisher started: model=%s, frame=%s -> %s",
            self.model_name,
            self.odom_frame,
            self.base_frame,
        )

        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            if self.current_pose is not None:
                self.publish_odom()
            rate.sleep()

    def model_states_cb(self, msg):
        try:
            idx = msg.name.index(self.model_name)
            self.current_pose = msg.pose[idx]
            self.current_twist = msg.twist[idx]
        except ValueError:
            rospy.logwarn_throttle(
                5.0, "Model '%s' not found in /gazebo/model_states", self.model_name
            )

    def publish_odom(self):
        pose = self.current_pose
        twist = self.current_twist

        now = rospy.Time.now()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose = pose

        # Gazebo 的 twist 通常是世界坐标系下的线速度，需要转到 base_link 坐标系
        q = pose.orientation
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        vx_world = twist.linear.x
        vy_world = twist.linear.y
        odom.twist.twist.linear.x = vx_world * cos_yaw + vy_world * sin_yaw
        odom.twist.twist.linear.y = -vx_world * sin_yaw + vy_world * cos_yaw
        odom.twist.twist.linear.z = twist.linear.z
        odom.twist.twist.angular = twist.angular

        # 仿真 ground truth，协方差给很小
        odom.pose.covariance = [
            1e-5, 0, 0, 0, 0, 0,
            0, 1e-5, 0, 0, 0, 0,
            0, 0, 1e10, 0, 0, 0,
            0, 0, 0, 1e10, 0, 0,
            0, 0, 0, 0, 1e10, 0,
            0, 0, 0, 0, 0, 1e-5,
        ]
        odom.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.br.sendTransform(t)


if __name__ == "__main__":
    try:
        GazeboOdomPublisher()
    except rospy.ROSInterruptException:
        pass
