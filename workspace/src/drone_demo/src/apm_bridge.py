#!/usr/bin/env python
# license removed for brevity
import rospy, tf
from geometry_msgs.msg import PoseStamped

pose1 = PoseStamped()
pose2 = PoseStamped()
pose3 = PoseStamped()

def callback1(data):
    global pose1
    pose1 = data

def callback2(data):
    global pose2
    pose2 = data
    
def callback3(data):
    global pose3
    pose3 = data
    
def talker():
    rospy.init_node('bridge', anonymous=True)
    # drone 1
    drone1_pub1 = rospy.Publisher('/drone1/mavros/mocap/pose', PoseStamped, queue_size=1)
    drone1_pub2 = rospy.Publisher('/drone1/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("/mocap_node/Robot_1/pose", PoseStamped, callback1)
    
    # drone 2
    drone2_pub1 = rospy.Publisher('/drone2/mavros/mocap/pose', PoseStamped, queue_size=1)
    drone2_pub2 = rospy.Publisher('/drone2/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("/mocap_node/Robot_2/pose", PoseStamped, callback2)
    
    # drone 3
    drone3_pub1 = rospy.Publisher('/drone3/mavros/mocap/pose', PoseStamped, queue_size=1)
    drone3_pub2 = rospy.Publisher('/drone3/mavros/vision_pose/pose', PoseStamped, queue_size=1)
    rospy.Subscriber("/mocap_node/Robot_3/pose", PoseStamped, callback3)
    
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        # drone 1
        drone1_pub1.publish(pose1)
        drone1_pub2.publish(pose1)
        
        # drone 2
        drone2_pub1.publish(pose2)
        drone2_pub2.publish(pose2)
        
        # drone 3
        drone3_pub1.publish(pose3)
        drone3_pub2.publish(pose3)
             
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass