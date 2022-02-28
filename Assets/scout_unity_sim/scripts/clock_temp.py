#! /usr/bin/env python

import rospy
import rospy
import sensor_msgs.msg

def clbk_odom(msg):
    global vel_pub
    global i
    t = msg
    t.header.stamp = rospy.Time.now()
    t.header.seq = i
    i = i+1
    t.header.frame_id = "velodyne"
    vel_pub.publish(t)
    

def main():
    global vel_pub
    global i 
    i = 0
    rospy.init_node("scan_FIXER")

    #defining pubs and subs
    odom_sub = rospy.Subscriber("/velodyne_points", sensor_msgs.msg.PointCloud2, clbk_odom)
    vel_pub = rospy.Publisher("/velodyne_points_correct", sensor_msgs.msg.PointCloud2, queue_size=100)

    r =rospy.Rate(20)
    while not rospy.is_shutdown():
        # Run this loop at about 10Hz
        r.sleep()
    #rospy.spin()
        
if __name__=="__main__":
    main()




