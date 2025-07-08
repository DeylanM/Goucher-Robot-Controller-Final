#kimbleRules Goucher Robotics Starter File
#Created by Deylan Mondeel with assistance from Sonny Kennison and Xavier Rivers
#This file aims to make getting started with the kimble robot

import sys; #path.append('/opt/ros/noetic/lib/python3/dist-packages/')
from os import environ
environ['ROS_MASTER_URI'] = "http://kimble:11311"
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import copy
import numpy as np
import cv2
import argparse
import time
a=0;
pinkY = 0;
pinkX = 0;
pinkDepth = -100;
timeToMove = False;
from geometry_msgs.msg import Twist
robot = moveit_commander.RobotCommander()
rospy.init_node('move_group_python_interface_tutorial',anonymous=True)

group = moveit_commander.MoveGroupCommander("gripper")
manip = moveit_commander.MoveGroupCommander("manipulator")

display_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)


#Initilizes values and grabs the link to the end affector and the current joint settings of each arm joint

group.clear_pose_targets()
endAffector = group.get_end_effector_link()
endJoints = group.get_current_joint_values() 
armJoints = manip.get_current_joint_values()
bottomOutlet = manip.get_current_joint_values()

#The arm has six joints and their individual orientations can be set as shown below, to easily find the 
#desired position RVIZ can be used.

armJoints[0] = 0
armJoints[1] = 1.9
armJoints[2] = 2.060
armJoints[3] = 1.550
armJoints[4] = 1.330
armJoints[5] = -1.550

bottomOutlet[0] = -0.052
bottomOutlet[1] = -2.090
bottomOutlet[2] = -0.160
bottomOutlet[3] = 1.550
bottomOutlet[4] = -0.367
bottomOutlet[5] = -1.550

	 
startpose = group.get_current_pose(endAffector).pose


#The ImageListener class handles what the camera sees and the information that it can process, there are two subroutines being run currently, ImageColorCallBack and ImageDepthCallback, ImageColorCallBack can be set to look for a color within view and then find its center, ImageDepthCallback returns the depth value between a pixel and the camera



### Camera Functions ###
class ImageListener:
	def __init__(self,topic, topic2):
		self.topic = topic
		self.topic2 = topic2
		self.bridge = CvBridge()
		self.sub = rospy.Subscriber(topic2,msg_Image,self.imageDepthCallback)
		self.sub = rospy.Subscriber(topic,msg_Image,self.imageColorCallback)
		self.depth = 0
		
		
	def imageColorCallback(self,data):
		try:
			lower_red = np.array([120, 80, 100], np.uint8) 
			upper_red= np.array([180, 255, 255], np.uint8)
			cv_image1 = self.bridge.imgmsg_to_cv2(data,data.encoding)
			cv_image = cv2.cvtColor(cv_image1, cv2.COLOR_BGR2HSV)
			mask = cv2.inRange(cv_image, lower_red, upper_red)
			detected_output = cv2.bitwise_and(cv_image, cv_image, mask = mask)
			contours, hierarchies = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
  			

			blank = np.zeros(cv_image.shape[:2], 
                 		dtype='uint8')
 
			cv2.drawContours(blank, contours, -1, 
                		(255, 0, 0), 1)
                	

			for i in contours:
    				M = cv2.moments(i)
    				if M['m00'] != 0:
    					cx = int(M['m10']/M['m00'])
    					cy = int(M['m01']/M['m00'])
    					cv2.drawContours(detected_output, [i], -1, (0, 255, 0), 2)
    					cv2.circle(detected_output, (cx, cy), 7, (0, 0, 255), -1)
    					cv2.putText(detected_output, "center", (cx - 20, cy - 20),
    					cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    					global pinkY
    					pinkY = cy
    					global pinkX
    					pinkX = cx

        				#print(cx)
        				#print(cy)

        				        					
		except CvBridgeError as e:
			print(e)
			return
			
	def imageDepthCallback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data,data.encoding)
			pix = (data.width/2, data.height/2)
			depth = cv_image[int(pix[1]),int(pix[0])]
			global pinkDepth
			pinkDepth = cv_image[int(pinkX), int(pinkY)]
			#print(pinkDepth)
			self.depth = depth
			
		except CvBridgeError as e:
			#print(e)
			return









		


#Sends a signal to any rviz programs you have open to visually show the movement being attemted		
def display_trajectory(plan):
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()
	display_trajectory.trajectory_start = robot.get_current_state()
	display_trajectory.trajectory.append(plan)
	display_publisher.publish(display_trajectory)

### Base Functions ###

#Moves the robot base by x1, x2, x3
	#x1 = linear velocity
	#x2 = horizontal velocity
	#x3 = angular Velocity
	

def move_robot(linear_vel,angular_vel):

	
	twist_msg = Twist()
	twist_msg.linear.x = linear_vel
	twist_msg.angular.z = angular_vel
	cmd_drive_pub.publish(twist_msg)

#Wander has the robot base move in a direction then stop and move in another direction when it gets too close to an object
def wander():
	while (tooClose == False and (float(rospy.Time().now().secs) - globalStart) < 60):
		start = float(rospy.Time().now().secs)
		move_robot(0.1,0.0)
		rate.sleep()
		if listener.depth < 350:
			tooClose = True
		while (float(rospy.Time().now().secs) - start) < random.randrange(200):
			move_robot(0.0,.5)
			rate.sleep()
			tooClose = False


#move_color_depth aims to position the robot base into an optimal position for the arm to grab an object based on its distance to the object
#the specific pinkDepth you want the robot to stop at can be set depending on light conditions		
def move_color_depth():
	goodDistance = False
	while goodDistance == False:
		if pinkDepth == 0:
			move_robot(0.1,0)
			time.sleep(2)
		elif pinkDepth > 535:
			move_robot(0.1,0)

		elif pinkDepth < 515:
			move_robot(-0.3,0)

		else:
			goodDistance = True
	return
	
### Arm Functions ###
#Sets the arms position to the bottomOutlet position documented in RVIZ and above
def lower_arm():
	manip.clear_pose_targets()
	manip.set_joint_value_target(bottomOutlet)
	manip.go()
	time.sleep(3)

#Sets the arms position to the armJoints array listed above or "stow" position defined in RVIZ
def raise_arm():
	manip.clear_pose_targets()
	manip.set_joint_value_target(armJoints)
	manip.go()
	time.sleep(3)

#pointPink scans for a pink object and then has the robot arm point torwards the center of the object
def pointPink():
	healthyNamingConventionX = pinkX
	healthyNamingConventionY = pinkY
	posY = (healthyNamingConventionX - 320)/640
	posZ = (healthyNamingConventionY + 240)/480
	startpose = manip.get_current_pose().pose
	pose = geometry_msgs.msg.Pose()
	pose.orientation = startpose.orientation
	pose.position.x = startpose.position.x + .45
	pose.position.y = startpose.position.y - posY
	pose.position.z = posZ - .35	
	manip.set_pose_target(pose)
	waypoints = []
	waypoints.append(startpose)
	waypoints.append(copy.deepcopy(pose))
	(plan,fraction) = manip.compute_cartesian_path(waypoints,0.01,0.0)
	manip.go()
	
#findpink causes the robot to slowly turn torwards the right untill the center of a pink object is in the center of the camera, before swinging back slightly to adjust for any momentum based errors	
def find_pink():
	x = False
	goalPost = 320
	while x != True:
		move_robot(0,0.1)
		if checkTolerance(pinkX,goalPost,10):
			x = True
			print("Center Found")
			return

### End Effector Functions ###
#Closes the hand of the gripper end effector
def close_grip():
	group.clear_pose_targets()
	pose = geometry_msgs.msg.Pose()
	pose.orientation.x = startpose.orientation.x
	pose.orientation.y = startpose.orientation.y
	pose.orientation.z = startpose.orientation.z
	pose.position.x = startpose.position.x
	pose.position.y = startpose.position.y
	pose.position.z = startpose.position.z
	endJoints[0] = 0
	group.set_joint_value_target(endJoints)
	waypoints = []
	waypoints.append(startpose)
	waypoints.append(copy.deepcopy(pose))
	(plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
	display_trajectory(plan)
	group.go()
	time.sleep(3)

#Opens the hand of the gripper end effector
def open_grip():
	pose = geometry_msgs.msg.Pose()
	pose.orientation.x = startpose.orientation.x
	pose.orientation.y = startpose.orientation.y
	pose.orientation.z = startpose.orientation.z
	pose.position.x = startpose.position.x
	pose.position.y = startpose.position.y
	pose.position.z = startpose.position.z
	endJoints[0] = .960
	group.set_joint_value_target(endJoints)
	waypoints = []
	waypoints.append(startpose)
	waypoints.append(copy.deepcopy(pose))
	(plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
	display_trajectory(plan)
	group.go()
	time.sleep(1)
	
#Turn joint 6 of the main arm to rotate the end effector, input a rotational position between -2.5 and 2.5 in radians
 
def turn_grip(directionRad):
	turnDirection = directionRad
	manip.clear_pose_targets()
	pose = geometry_msgs.msg.Pose()
	pose.orientation.x = startpose.orientation.x
	pose.orientation.y = startpose.orientation.y
	pose.orientation.z = startpose.orientation.z
	pose.position.x = startpose.position.x
	pose.position.y = startpose.position.y
	pose.position.z = startpose.position.z
	armJoints[5] = turnDirection
	manip.set_joint_value_target(armJoints)
	waypoints = []
	waypoints.append(startpose)
	waypoints.append(copy.deepcopy(pose))
	(plan,fraction) = group.compute_cartesian_path(waypoints,0.01,0.0)
	display_trajectory(plan)
	manip.go()
	time.sleep(1)
	
### Useful Functions ###

#Function that allows you to quickly add a level of tolerence, which is important when working with impercice tools such as the camera
def checkTolerance(value, target, tolerance):
	lower_bound = value - tolerance
	upper_bound = value + tolerance
	if target >= lower_bound and target <= upper_bound:
		return True
	else:
		return False


cmd_drive_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
start = float(rospy.Time().now().secs)
rate = rospy.Rate(10)
topic = '/realsense/color/image_raw'
topic2 = '/realsense/depth/image_rect_raw'
listener = ImageListener(topic, topic2)


#Your Code Here

open_grip()
find_pink()
move_color_depth()
lower_arm()
close_grip()
raise_arm()
open_grip()










