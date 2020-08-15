#!/usr/bin/env python
import rospy
import matplotlib.pyplot
from geometry_msgs.msg import PointStamped
    
def main():
    print "Intializing plotter node ..."
    rospy.init_node("plotter")
    pub_point = rospy.Publisher("/object_point", PointStamped, queue_size=10)
    loop = rospy.Rate(20);
    msg_point = PointStamped()
    msg_point.header.frame_id = "left_arm_link7"
    msg_point.point.x = 0.20
    msg_point.point.y = 0
    msg_point.point.z = 0
    while not rospy.is_shutdown():
        msg_point.header.stamp = rospy.Time.now()
        pub_point.publish(msg_point)
        loop.sleep()
        
if __name__ == '__main__':
    main()
