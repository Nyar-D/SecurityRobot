#!/usr/bin/python3
## 小车控制节点，订阅/cmd_vel，监听geometry_msgs

import sys, os
import math
import threading
import rospy, tf.transformations, tf2_ros
from robot_control.msg import WheelSpeed
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import time

sys.path.append(os.path.split(os.path.realpath(__file__))[0])
import pid as pctl


class BaseController():
    def __init__(self, nodeName) -> None:
        pub_t0 = 50  # 里程计发布频率

        self.x = 0.0;                   # 初始位置x的坐标
        self.y = 0.0;                   # 初始位置y的坐标
        self.th = 0.0;                  # 初始位置的角度
        self.vx = 0.0;                  # x方向的初始速度
        self.vy = 0.0;                  # y方向的初始速度
        self.vth = 0.0;                 # 初始角速度
        max = math.radians(360)

        # ROS初始化
        rospy.init_node(nodeName, anonymous=True)
        rospy.Subscriber('cmd_vel', Twist, self.set_speed)
        self.vel_pub = rospy.Publisher("wheel_vel", WheelSpeed, queue_size=1000)
        self.odom_pub = rospy.Publisher("odom_wheel", Odometry, queue_size=1000)
        self.odom_broadcaster = tf2_ros.TransformBroadcaster()

        # 消息发送的时间间隔
        rate = rospy.Rate(pub_t0)
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        
        # ======================================================
        th_print = threading.Thread(target=self.print_speed)
        th_print.setDaemon(True)
        th_print.start()
        # ======================================================
        
        while not rospy.is_shutdown():
            self.current_time = rospy.Time.now()
            
            # 计算机器人当前的线速度和角速度，咱的机器人只有linear.x和angular.z
            self.vy = 0
            self.vx = ((pctl.cur_lv + pctl.cur_rv) / 2)
            self.vth = (pctl.cur_rv - pctl.cur_lv) / pctl.l
            
            # 航迹推演：根据机器人速度计算里程计（位置、普通移动模式）
            dt = (self.current_time - self.last_time).to_sec()
            delta_x = (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * dt
            delta_y = (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * dt
            delth_th = self.vth * dt
            
            self.x += delta_x
            self.y += delta_y
            self.th += delth_th
            self.th = self.th if self.th < max else self.th - max
            self.th = self.th if self.th > -max else self.th + max

            # 发布
            # self.broadcast_transform()
            self.publish_odometry()
            # self.publish_wheelSpeed()
            
            # 更新计算参数
            self.last_time = self.current_time

            rate.sleep()


    def set_speed(self, data):
        # v: 线速度 w: 角速度 l: 轮胎距离
        v = data.linear.x
        w = data.angular.z
        l = pctl.l

        # 计算左右轮速度 velocity
        vr = (2 * v + l * w)/ 2
        vl = (2 * v - vr)
        # rospy.loginfo(f"左右轮速度: {vl} {vr}")

        # 调整PID的目标速度
        pctl.trg_lv = vl
        pctl.trg_rv = vr
        # print(pctl.l_pid.totalErr, pctl.r_pid.totalErr)
        pctl.l_pid.totalErr = 0
        pctl.r_pid.totalErr = 0
        
        
    def broadcast_transform(self):
        # 广播坐标系转换TF: /odom to /base_footprint
        odom_tfs = TransformStamped()
        odom_tfs.header.stamp = self.current_time
        odom_tfs.header.frame_id = "odom"
        odom_tfs.child_frame_id = "base_footprint"

        odom_tfs.transform.translation.x = self.x
        odom_tfs.transform.translation.y = self.y
        odom_tfs.transform.translation.z = 0.0 
        qtn = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom_tfs.transform.rotation.x = qtn[0]
        odom_tfs.transform.rotation.y = qtn[1]
        odom_tfs.transform.rotation.z = qtn[2]
        odom_tfs.transform.rotation.w = qtn[3]
            
        self.odom_broadcaster.sendTransform(odom_tfs)
        
        
    def publish_odometry(self):
        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        
        # 设置position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        qtn = tf.transformations.quaternion_from_euler(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = qtn[0]
        odom_msg.pose.pose.orientation.y = qtn[1]
        odom_msg.pose.pose.orientation.z = qtn[2]
        odom_msg.pose.pose.orientation.w = qtn[3]
        odom_msg.pose.covariance = [1e-3, 0, 0, 0, 0, 0,
                                    0, 1e-3, 0, 0, 0, 0,
                                    0, 0, 1e6, 0, 0, 0,
                                    0, 0, 0, 1e6, 0, 0,
                                    0, 0, 0, 0, 1e6, 0,
                                    0, 0, 0, 0, 0, 1e3]
        
        # 设置velocity
        # odom_msg.twist.twist.linear.x = self.vx
        # odom_msg.twist.twist.linear.y = self.vy
        # odom_msg.twist.twist.angular.z = self.vth
        # odom_msg.twist.covariance = [1e-3, 0, 0, 0, 0, 0,
        #                              0, 1e-3, 0, 0, 0, 0,
        #                              0, 0, 1e6, 0, 0, 0,
        #                              0, 0, 0, 1e6, 0, 0,
        #                              0, 0, 0, 0, 1e6, 0,
        #                              0, 0, 0, 0, 0, 1e3]

        self.odom_pub.publish(odom_msg)
    
    
    def publish_wheelSpeed(self):
        # 发布车轮速度消息
        wheel_speed = WheelSpeed()
        wheel_speed.current.left = pctl.cur_lv
        wheel_speed.current.right = pctl.cur_rv
        wheel_speed.target.left = pctl.trg_lv
        wheel_speed.target.right = pctl.trg_lv
        self.vel_pub.publish(wheel_speed)
        
        
    def print_speed(self):
        # print(f"x:{self.x:.3f} y:{self.y:.3f} th:{self.th:.3f}")
        while True:
            if pctl.cur_lv != 0:
                rospy.loginfo(f"线角:{self.vx:.3f} {self.vth:.3f} PWM:{pctl.l_pid.setPwm:.0f} {pctl.r_pid.setPwm:.0f} 当前(cm/s):{pctl.cur_lv * 100:.2f} {pctl.cur_rv * 100:.2f}")
            time.sleep(0.1)


if __name__ == '__main__':
    try:
        pctl.start()
        controller_node = BaseController('base_controller')
    except KeyboardInterrupt:
        pctl.isExit = 1
        rospy.signal_shutdown('KeyboardInterrupt')
        exit()
    