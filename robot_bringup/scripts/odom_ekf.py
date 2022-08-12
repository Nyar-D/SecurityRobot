#!/usr/bin/env python
## 消息类型转换节点，将ekf发布的PoseWithCovarianceStamped转换成Odometry类型的消息，供move_base订阅

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
 
class OdomEKF():
   def __init__(self, nodeName):
       # Give the node a name
       rospy.init_node(nodeName, anonymous=False)
 
       # Publisher of type nav_msgs/Odometry
       self.ekf_pub = rospy.Publisher('output', Odometry, queue_size=10)
       
       # Wait for the /odom_combined topic to become available
       rospy.wait_for_message('input', PoseWithCovarianceStamped)
       
       # Subscribe to the /odom_combined topic
       rospy.Subscriber('input', PoseWithCovarianceStamped, self.pub_ekf_odom)
       
       rospy.loginfo("Publishing combined odometry on /odom_ekf")
       
   def pub_ekf_odom(self, msg):
       odom = Odometry()
       odom.header = msg.header
       odom.header.frame_id = 'odom'
       odom.child_frame_id = 'base_footprint'
       odom.pose = msg.pose
       
       self.ekf_pub.publish(odom)
       
if __name__ == '__main__':
    try:
        OdomEKF("odom_ekf")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
