#!/usr/bin/env python3
import rospy

from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest
from ur_dashboard_msgs.srv import Load

gripper_cmd = String()
mode_updated = False
# get the command/mode message
def get_cmd(cmd):
	global gripper_cmd
	global mode_updated
	gripper_cmd = cmd
	print(gripper_cmd.data)
	mode_updated = True

	
if __name__ == '__main__':
	# define the node and subcribers and publishers
	rospy.init_node('gripper_control', anonymous = True)
	# define a subscriber to read images
	img_sub = rospy.Subscriber("/gripper_cmd", String, get_cmd) 

	# Check if all the services are available
	print('waiting for the services...not connected to the robot driver!')
	
	rospy.wait_for_service('/ur_hardware_interface/dashboard/load_program')
	print('LOAD Program service available')
	
	rospy.wait_for_service('/ur_hardware_interface/dashboard/play')
	print('PLAY service available')
	
	# set the loop frequency
	rate = rospy.Rate(10)

	try:
		load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
		program_request = 'ros_connect.urp'
		resp1 = load_urp(program_request) 
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)	
	while not rospy.is_shutdown():
		if mode_updated:
			if gripper_cmd.data == 'close':
				try:
					# load the close_gripper.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'close_gripper.urp'
					resp1 = load_urp(program_request) 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request) 
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)
			elif gripper_cmd.data == 'open':
				try:
					# load the open_gripper.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'open_gripper.urp'
					resp1 = load_urp(program_request) 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request) 
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)

			elif gripper_cmd.data == 'waypoint':
				try:
					# load the ros_connect.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'ros_connect.urp'
					resp1 = load_urp(program_request) 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request) 
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)
			else:
				print('The selected mode is not available. Select from: \"close\", \"open\", \"waypoint\" modes.')
			
			mode_updated = False
		# pause until the next iteration			
		rate.sleep()

