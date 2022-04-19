#!/usr/bin/env python3
import rospy

from std_msgs.msg import UInt8
from std_srvs.srv import Trigger, TriggerRequest
from ur_dashboard_msgs.srv import Load

gripper_cmd = UInt8()
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
	# define a publisher for providing feedback to the other codes
	task_status_pub = rospy.Publisher('/gripper_robot_status', UInt8, queue_size = 10)
	
	# define a subscriber to read commands
	cmd_sub = rospy.Subscriber("/gripper_cmd", UInt8, get_cmd) 

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
		# play the code on teach pendant
		play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
		request = TriggerRequest()
		resp1 = play_urp(request) 
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)
	feedback_msg = UInt8()
	while not rospy.is_shutdown():
		if mode_updated:
			if gripper_cmd.data == 2:
				try:
					# load the close_gripper.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'close_gripper.urp'
					resp1 = load_urp(program_request)
					if resp1.success:
						print('Loaded the CLOSE gripper program') 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request) 
					if resp1.success:
						print('CLOSING THE GRIPPER!') 
					rospy.sleep(1.4)
					print('Gripper CLOSED!')
					feedback_msg.data = 5 # 2 for the original request + 3 for the success
					task_status_pub.publish(feedback_msg)					
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)
			elif gripper_cmd.data == 1:
				try:
					# load the open_gripper.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'open_gripper.urp'
					resp1 = load_urp(program_request) 
					if resp1.success:
						print('Loaded the OPEN gripper program') 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request)
					if resp1.success:
						print('OPENING THE GRIPPER!') 
					rospy.sleep(1.4)
					print('Gripper OPENED!')
					feedback_msg.data = 4 # 1 for the original request + 3 for the success
					task_status_pub.publish(feedback_msg)					
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)

			elif gripper_cmd.data == 0:
				try:
					# load the ros_connect.urp from the teach pendant 
					load_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/load_program', Load)
					program_request = 'ros_connect.urp'
					resp1 = load_urp(program_request)
					if resp1.success:
						print('Loaded the ROBOT CONNECTION program') 
					# play the code on teach pendant
					play_urp = rospy.ServiceProxy('/ur_hardware_interface/dashboard/play', Trigger)
					request = TriggerRequest()
					resp1 = play_urp(request) 
					if resp1.success:
						print('CONNECTED TO THE ROBOT!') 
					rospy.sleep(0.5)
					feedback_msg.data = 3 # 0 for the original request + 3 for the success
					task_status_pub.publish(feedback_msg)					
				except rospy.ServiceException as e:
					print("Service call failed: %s"%e)
			else:
				print('The selected mode is not available. Select from: \"close\", \"open\", \"waypoint\" modes.')
			
			mode_updated = False
		# pause until the next iteration			
		rate.sleep()

