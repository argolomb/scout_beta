#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Vector3
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

import cv2
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class Subot_nav:
  def __init__(self):
    # MOVE BASE SETUP
    self.waypoints = []
    rospy.init_node('subot_inspection')
    self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    # CAMERA SETUP
    self.image = None
    self.bridge = CvBridge()
    rospy.Subscriber("/centerr200/camera/color/image_raw", Image , self.callback)

    time.sleep(1) 

  def callback(self, image_msg):
    self.image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")


  def add_waypoint(self, list):
    self.waypoints.append(list)


  def goal_move_base(self, pose_x, pose_y, pose_z, pose_w):
    msg_move_to_goal = PoseStamped()
    msg_move_to_goal.pose.position.x = pose_x 
    msg_move_to_goal.pose.position.y = pose_y
    msg_move_to_goal.pose.orientation.z = pose_z
    msg_move_to_goal.pose.orientation.w = pose_w
    msg_move_to_goal.header.frame_id = 'odom'
    rospy.sleep(1)
    self.move_base_pub.publish(msg_move_to_goal)
    

  def nav_into_points(self):
    for i in range(len(self.waypoints)):
      self.goal_move_base(self.waypoints[i][0], 
                          self.waypoints[i][1], 
                          self.waypoints[i][2], 
                          self.waypoints[i][3])

      rospy.wait_for_message("/move_base/result", 
                            MoveBaseActionResult, 
                            timeout=None)

      print("Passou pelo ponto " + str(i + 1))

      if self.waypoints[i][4] == 1:
        self.print_save_image(self.image, i) 


  def print_save_image(self, cv2_frame, number):
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(cv2_frame, 'ID do transformador: #' + str(number + 1) + ' Temp: 32 C' , (20, 60), font, 1, (50, 255, 50), 2) 
    cv2.putText(cv2_frame, str(time.ctime()), (20, 460), font, 1, (50, 255, 50), 2)

    cv2.imwrite(str(time.ctime()) + ' SUBOT_INSPECTION.jpg', cv2_frame)

    
# MAIN FUNCTION
if __name__ == '__main__':
  try:
    robot = Subot_nav()  
    # TRANSFORMADORES
    robot.add_waypoint([15.27, 35, -1.5708, 1.0, 1])
    robot.add_waypoint([24.48, 35, -1.5708, 1.0, 1])
    robot.add_waypoint([33.59, 35, -1.5708, 1.0, 1])
    # PONTO AUXILIAR
    robot.add_waypoint([0.0, 35, -1.5708, 1.0, 0])
    # PONTO INICIAL
    robot.add_waypoint([0.0, 0.0, 1.5708, 1.0, 0])
    
    robot.nav_into_points()

  except rospy.ROSInterruptException:
    pass		