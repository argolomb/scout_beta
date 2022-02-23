#!/usr/bin/env python3

import rospy

from ros_tcp_endpoint import TcpServer
from ros_tcp_endpoint.publisher import RosPublisher
from ros_tcp_endpoint.subscriber import RosSubscriber

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist, PoseStamped, TwistStamped
from sensor_msgs.msg import CompressedImage, LaserScan, JointState

def main(args=None):
    # Start the Server Endpoint
    tcp_ip_number = rospy.get_param("/unity_endpoint/tcp_ip")
    tcp_port_number = rospy.get_param("/unity_endpoint/tcp_port")
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name,tcp_ip=tcp_ip_number, tcp_port=tcp_port_number)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        "model_pose": RosPublisher("model_pose", PoseStamped, queue_size=1),
        # Sensors
        # camera
        "camera/color/image_raw/compressed": 
            RosPublisher("camera/color/image_raw/compressed", CompressedImage, queue_size=1),
        # lidar
        "base_scan": 
            RosPublisher("base_scan", LaserScan, queue_size=1)}
    )
    rospy.spin()


if __name__ == "__main__":
    main()
