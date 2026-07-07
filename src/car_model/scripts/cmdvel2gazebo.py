#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Quaternion
import tf.transformations

import tf2_ros
import geometry_msgs.msg

from sensor_msgs.msg import JointState

class CmdVel2Gazebo:

    def __init__(self):
        rospy.init_node('cmdvel2gazebo', anonymous=True)
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)   #接收速度和角速度
        rospy.Subscriber('/cmd_vel_k', Twist, self.callback_k, queue_size=1)   #接收速度和k
        rospy.Subscriber('/smart/joint_states', JointState, self.callback_joint_states, queue_size=1)   #接收各个关节的状态
        self.pub_whell_front_L = rospy.Publisher('/smart/front_left_svelocity_controller/command', Float64, queue_size=1)
        self.pub_steer_front_L = rospy.Publisher('/smart/front_left_str_controller/command', Float64, queue_size=1)
        self.pub_whell_front_R = rospy.Publisher('/smart/front_right_velocity_controller/command', Float64, queue_size=1)
        self.pub_steer_front_R = rospy.Publisher('/smart/front_right_str_controller/command', Float64, queue_size=1)
        self.pub_whell_rear_L = rospy.Publisher('/smart/rear_left_velocity_controller/command', Float64, queue_size=1)
        self.pub_steer_rear_L = rospy.Publisher('/smart/rear_left_str_controller/command', Float64, queue_size=1)
        self.pub__whell_rear_R = rospy.Publisher('/smart/rear_right_velocity_controller/command', Float64, queue_size=1)
        self.pub__steer_rear_R = rospy.Publisher('/smart/rear_right_str_controller/command', Float64, queue_size=1)
        #加入里程计
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0, 0.0, 0.0]  # 假设线速度为0.1 m/s
        self.orientation = [0.0, 0.0, 0.0, 1.0]  # 初始姿态为无旋转
        self.boby_w = 0
        self.boby_v = 0
        # initial velocity and tire angle are 0
        self.x = 0
        self.z = 0

        # car Wheelbase (in m)轴距
        self.L = 1.868

        # car Tread   轮距
        self.T_front = 1.284  
        self.T_rear = 1.284 #1.386

        # how many seconds delay for the dead man's switch
        self.timeout=rospy.Duration.from_sec(0.2)
        self.lastMsg=rospy.Time.now()

        # maximum steer angle of the "inside" tire 转角大小
        self.maxsteerInside=0.6

        # turning radius for maximum steer angle just with the inside tire
        # tan(maxsteerInside) = wheelbase/radius --> solve for max radius at this angle
        rMax = self.L/math.tan(self.maxsteerInside)

        # radius of inside tire is rMax, so radius of the ideal middle tire (rIdeal) is rMax+treadwidth/2
        rIdeal = rMax+(self.T_front/2.0)

        # maximum steering angle for ideal middle tire
        # tan(angle) = wheelbase/radius
        self.maxsteer=math.atan2(self.L,rIdeal)

        # loop
        rate = rospy.Rate(10) # run at 10Hz
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()
        

    def callback(self,data):
        self.x = data.linear.x 
        self.z = data.angular.z
        self.lastMsg = rospy.Time.now()
    
    def callback_k(self,data):
        self.x = data.linear.x 
        self.z = data.linear.x*data.angular.z
        if self.z>10:
            self.x = 0
            self.z = 0.2
        self.lastMsg = rospy.Time.now()

    def callback_joint_states(self,data):
        if not data.effort :
            print("不处理")
        else:
            front_right_wheel = data.velocity[3]
            front_left_wheel = data.velocity[1]
            rear_left_wheel = data.velocity[5]
            rear_right_wheel = data.velocity[7]
            front_left_steering = data.position[0]
            front_right_steering = data.position[2]
            rear_left_steering = data.position[4]
            rear_right_steering = data.position[6]
            print("反馈的1frontr转角:",  front_left_steering)
            # print("反馈的力:",  data.effort[0])
        #走直线或停
            if -0.00873<front_left_steering<0.00873 and -0.00873<front_right_steering<0.00873 and -0.00873<rear_left_steering<0.00873 and -0.00873<rear_right_steering<0.00873 :
                self.boby_w = 0
                self.boby_v = (front_right_wheel+front_left_wheel+rear_left_wheel+rear_right_wheel)/4*0.15
                print("走直线的v:", self.boby_v)
                print("走直线的w:", self.boby_w)
            else:
                lun1_R =  0.4615/math.sin(front_left_steering)
                boby_w_1 = front_left_wheel*0.15/lun1_R
                boby_v_1 = ((0.4615/math.tan(front_left_steering)+ 0.4)*boby_w_1)
                print("1轮速度:", boby_v_1)
                print("The answer is 1轮反解算v:", boby_v_1)
                print("The answer is 1轮反解算w:", boby_w_1)
                lun2_R =  0.4615/math.sin(front_right_steering)
                boby_w_2 = front_right_wheel*0.15/lun2_R
                boby_v_2 = ((0.4615/math.tan(front_right_steering)- 0.4)*boby_w_2)
                print("The answer is 2轮反解算v:", boby_v_2)
                print("The answer is 2轮反解算w:", boby_w_2)

                lun3_R =  -0.4725/math.sin(rear_left_steering)
                boby_w_3 = rear_left_wheel*0.15/lun3_R
                boby_v_3 = ((-(0.4725/math.tan(rear_left_steering))+0.4)*boby_w_3)
                print("The answer is 3轮反解算v:", boby_v_3)
                print("The answer is 3轮反解算w:", boby_w_3)

                lun4_R =  -0.4725/math.sin(rear_right_steering)
                boby_w_4 = rear_right_wheel*0.15/lun4_R
                boby_v_4 = ((-(0.4725/math.tan(rear_right_steering))-0.4)*boby_w_4)
                print("The answer is 4轮反解算v:", boby_v_4)
                print("The answer is 4轮反解算w:", boby_w_4)

                self.boby_v = (boby_v_1 + boby_v_2 + boby_v_3 + boby_v_4)/4
                self.boby_w = (boby_w_1 + boby_w_2 + boby_w_3 + boby_w_4)/4

    def publish(self):
        # The self.z is the delta angle in radians of the imaginary front wheel of ackerman model.
        if self.z != 0:
            # self.v is the linear *velocity*正常转向
            ###############4轮8驱转向
            """ R = self.x/self.z
            w = self.z
            print("暂停后的w:", w)
            print("暂停后的v:", self.x)
            ################
            msgfrontL_steer = Float64()
            msgfrontL_steer.data = math.atan(0.4615/(R - 0.4))
            msgfrontL_v = Float64()
            msgfrontL_v.data = 0.461/math.sin(msgfrontL_steer.data)*w/0.15

            msgfrontR_steer = Float64()
            msgfrontR_steer.data = math.atan(0.4615/(R + 0.4))
            msgfrontR_v = Float64()
            msgfrontR_v.data = 0.4615/math.sin(msgfrontR_steer.data)*w/0.15

            msgrearL_steer = Float64()
            msgrearL_steer.data = -math.atan(0.4725/(R -  0.4))
            msgrearL_v = Float64()
            msgrearL_v.data = -0.4725/math.sin(msgrearL_steer.data)*w/0.15

            msgrearR_steer = Float64()
            msgrearR_steer.data = -math.atan(0.4725/(R +  0.4))
            msgrearR_v = Float64()
            msgrearR_v.data = -0.4725/math.sin(msgrearR_steer.data)*w/0.15
            
            self.pub_steer_front_L.publish(msgfrontL_steer)
            self.pub_whell_front_L.publish(msgfrontL_v)

            self.pub_steer_front_R.publish(msgfrontR_steer)
            self.pub_whell_front_R.publish(msgfrontR_v)

            self.pub_steer_rear_L.publish(msgrearL_steer)
            self.pub_whell_rear_L.publish(msgrearL_v )

            self.pub__steer_rear_R.publish(msgrearR_steer)
            self.pub__whell_rear_R.publish(msgrearR_v)

            lun1_R =  0.4615/math.sin(msgfrontL_steer.data)
            self.boby_w = msgfrontL_v.data*0.15/lun1_R
            self.boby_v = ((0.4615/math.tan(msgfrontL_steer.data)+ 0.4)*self.boby_w) 
            #boby_K = boby_w/boby_v
            print("The answer is 1frontL_steer转角:",  msgfrontL_steer.data) """
            ####################################阿克曼模型
            R = self.x/self.z
            w = self.z
            msgfrontL_steer = Float64()
            msgfrontL_steer.data = math.atan(0.934/(R - 0.4));
            msgfrontL_v = Float64()
            msgfrontL_v.data = 0; 
            print("The answer is lun1_jiaod:", msgfrontL_steer.data * (180.0 / math.pi))

            msgfrontR_steer = Float64()
            msgfrontR_steer.data = math.atan(0.934/(R + 0.4));
            msgfrontR_v = Float64()
            msgfrontR_v.data = 0;
            print("The answer is lun2_jiaod:", msgfrontR_steer.data * (180.0 / math.pi))

            msgrearL_steer = Float64()
            msgrearL_steer.data = 0;
            msgrearL_v = Float64()
            msgrearL_v.data = (R-0.4)*w/0.15;

            msgrearR_steer = Float64()
            msgrearR_steer.data = 0;
            msgrearR_v = Float64()
            msgrearR_v.data = (R+0.4)*w/0.15;
            self.pub_steer_front_L.publish(msgfrontL_steer)
            self.pub_whell_front_L.publish(msgfrontL_v)

            self.pub_steer_front_R.publish(msgfrontR_steer)
            self.pub_whell_front_R.publish(msgfrontR_v)

            self.pub_steer_rear_L.publish(msgrearL_steer)
            self.pub_whell_rear_L.publish(msgrearL_v )

            self.pub__steer_rear_R.publish(msgrearR_steer)
            self.pub__whell_rear_R.publish(msgrearR_v)

            lun3_R =  0.934/math.tan(msgfrontL_steer.data)
            boby_w = msgrearL_v.data*0.15/lun3_R
            boby_v = ((lun3_R+0.4)*boby_w)
            boby_K = boby_w/boby_v
            print("The answer is v:", boby_v)
            print("The answer is w:", boby_w)
            print("The answer is K:", boby_K)
            print("The answer is lun4_jiaodu:",  msgrearR_steer.data)
            print("The answer is lun4_shudu:",  msgrearR_v.data*0.15) 

            ###########差速模型分配(理论分配)
            """ R = self.x/self.z;
            msgfrontL_steer = Float64()
            msgfrontL_steer.data = 0;
            msgfrontL_v = Float64()
            msgfrontL_ang = math.atan(0.923/(R - 0.642));
            msgfrontL_sum_v =  0.923/math.sin(msgfrontL_ang)*self.z;
            msgfrontL_v.data = math.cos(msgfrontL_ang)*msgfrontL_sum_v/0.3;
           
            msgfrontR_steer = Float64()
            msgfrontR_steer.data = 0;
            msgfrontR_v = Float64()
            msgfrontR_ang = math.atan(0.923/(R + 0.642));
            msgfrontR_sum_v = 0.923/math.sin(msgfrontR_ang)*self.z;
            msgfrontR_v.data = math.cos(msgfrontR_ang)*msgfrontR_sum_v/0.3;
        
            msgrearL_steer = Float64()
            msgrearL_steer.data = 0;
            msgrearL_v = Float64()
            msgrearL_ang = -math.atan(0.945/(R - 0.642));
            msgrearL_sum_v = -0.945/math.sin(msgrearL_ang)*self.z ;
            msgrearL_v.data = math.cos(msgrearL_ang)*msgrearL_sum_v/0.3;
        
            msgrearR_steer = Float64()
            msgrearR_steer.data = 0;
            msgrearR_v = Float64()
            msgrearR_ang = -math.atan(0.945/(R + 0.642));
            msgrearR_sum_v = -0.945/math.sin(msgrearR_ang)*self.z;
            msgrearR_v.data = math.cos(msgrearR_ang)*msgrearR_sum_v/0.3;
            
            self.pub_steer_front_L.publish(msgfrontL_steer)
            self.pub_whell_front_L.publish(msgfrontL_v)

            self.pub_steer_front_R.publish(msgfrontR_steer)
            self.pub_whell_front_R.publish(msgfrontR_v)

            self.pub_steer_rear_L.publish(msgrearL_steer)
            self.pub_whell_rear_L.publish(msgrearL_v )

            self.pub__steer_rear_R.publish(msgrearR_steer)
            self.pub__whell_rear_R.publish(msgrearR_v) """


            ###############################4论差速（gazebo插件版）
            """ R = self.x/self.z;
            msgfrontL_steer = Float64()
            msgfrontL_steer.data = 0;
            msgfrontL_v = Float64()
            msgfrontL_ang = math.atan(0.923/(R - 0.642));
            msgfrontL_sum_v =  0.923/math.sin(msgfrontL_ang)*self.z;
            msgfrontL_v.data = self.x/0.3+-1*self.z*(0.642/0.3) ;
           
            msgfrontR_steer = Float64()
            msgfrontR_steer.data = 0;
            msgfrontR_v = Float64()
            msgfrontR_ang = math.atan(0.923/(R + 0.642));
            msgfrontR_sum_v = 0.923/math.sin(msgfrontR_ang)*self.z;
            msgfrontR_v.data = self.x/0.3+1*self.z*(0.642/0.3);
        
            msgrearL_steer = Float64()
            msgrearL_steer.data = 0;
            msgrearL_v = Float64()
            msgrearL_ang = -math.atan(0.945/(R - 0.642));
            msgrearL_sum_v = -0.945/math.sin(msgrearL_ang)*self.z ;
            msgrearL_v.data =self.x/0.3+-1*self.z*(0.642/0.3);
        
            msgrearR_steer = Float64()
            msgrearR_steer.data = 0;
            msgrearR_v = Float64()
            msgrearR_ang = -math.atan(0.945/(R + 0.642));
            msgrearR_sum_v = -0.945/math.sin(msgrearR_ang)*self.z;
            msgrearR_v.data = self.x/0.3+1*self.z*(0.642/0.3);
            
            self.pub_steer_front_L.publish(msgfrontL_steer)
            self.pub_whell_front_L.publish(msgfrontL_v)

            self.pub_steer_front_R.publish(msgfrontR_steer)
            self.pub_whell_front_R.publish(msgfrontR_v)

            self.pub_steer_rear_L.publish(msgrearL_steer)
            self.pub_whell_rear_L.publish(msgrearL_v )

            self.pub__steer_rear_R.publish(msgrearR_steer)
            self.pub__whell_rear_R.publish(msgrearR_v) """
            ##############

        else:
            # 停下or直线
            msgRear = Float64()
            msgRear.data = self.x/0.15;
            msgSteer = Float64()
            msgSteer.data = self.z

            self.pub_steer_front_L.publish(msgSteer)
            self.pub_whell_front_L.publish(msgRear)

            self.pub_steer_front_R.publish(msgSteer)
            self.pub_whell_front_R.publish(msgRear)

            self.pub_steer_rear_L.publish(msgSteer)
            self.pub_whell_rear_L.publish(msgRear)

            self.pub__steer_rear_R.publish(msgSteer)
            self.pub__whell_rear_R.publish(msgRear)
            """ self.boby_w = 0
            self.boby_v = self.x """
            # print("The answer is v:", msgRear.data*0.15)
            # print("The answer is w:", msgSteer.data)    
        ####################里程计
        (_, _, yaw) = tf.transformations.euler_from_quaternion([self.orientation[0],self.orientation[1],self.orientation[2],self.orientation[3]])
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        if (self.boby_w!=0) and (self.boby_v==0) :
            yaw = yaw+self.boby_w*(1.0 / 10.0)
        else:
            self.position[0] += self.boby_v *math.cos(yaw)* (1.0 / 10.0)
            self.position[1] += self.boby_v *math.sin(yaw)* (1.0 / 10.0)
            # print("The answer is x轴:", self.position[0])
            # print("The answer is y轴:", self.position[1])
            yaw = yaw+self.boby_w*(1.0 / 10.0)
            print("The answer is 角度:",  math.degrees(yaw))
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.orientation[0] = quaternion[0]
        self.orientation[1] = quaternion[1]
        self.orientation[2] = quaternion[2]
        self.orientation[3] = quaternion[3]
        pose = Pose()
        pose.position.x = self.position[0]
        pose.position.y = self.position[1]
        pose.position.z = self.position[2]
        pose.orientation = Quaternion(*self.orientation)
        odom.pose.pose = pose
        twist = Twist()
        twist.linear.x = self.boby_v
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.boby_w
        odom.twist.twist = twist
        self.odom_pub.publish(odom)
        print("The answer is 反解算v:", self.boby_v)
        print("The answer is 反解算w:", self.boby_w)
        print("The answer is x轴:", self.position[0])
        print("The answer is y轴:", self.position[1])

        #加入tf
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        q = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)
       
        delta_last_msg_time = rospy.Time.now() - self.lastMsg
        msgs_too_old = delta_last_msg_time > self.timeout
        if msgs_too_old:
            self.x = 0
            self.z = 0
            msgRear = Float64()
            msgRear.data = self.x
            msgSteer = Float64()
            msgSteer.data = 0
            
            self.pub_steer_front_L.publish(msgSteer)
            self.pub_whell_front_L.publish(msgRear)

            self.pub_steer_front_R.publish(msgSteer)
            self.pub_whell_front_R.publish(msgRear)

            self.pub_steer_rear_L.publish(msgSteer)
            self.pub_whell_rear_L.publish(msgRear)

            self.pub__steer_rear_R.publish(msgSteer)
            self.pub__whell_rear_R.publish(msgRear)
            return



if __name__ == '__main__':
    try:
        CmdVel2Gazebo()
    except rospy.ROSInterruptException:
        pass




