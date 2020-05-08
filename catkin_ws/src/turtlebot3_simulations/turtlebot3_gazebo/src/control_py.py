#!/usr/bin/env python

import rospy 
import numpy as np
# from laser_assembler.srv import AssembleScans2
# from sensor_msgs.msg import PointCloud2 as pc2
from geometry_msgs.msg import Twist
from Math import pi
# from laser_geometry import LaserProjection

class control_py():
	"""docstringfor """
	def __init__(self,):
		self.vel_publisher = rospy.Publisher("cmd_vel",Twist,queue_size=1)
		# self.laserSub = rospy.Subscriber("scan",LaserScan,self.laserCallback)
		self.vel_msg = Twist()

	def controller(self):
		self.vel_msg.linear.x = 0 #np.random.uniform(0,0.1,1)
		self.vel_msg.linear.y = 0 #np.random.uniform(0,0.1,1)
		self.vel_msg.linear.z = 0		
		self.vel_msg.angular.x = 0
		self.vel_msg.angular.y = 0
		self.vel_msg.angular.z = 0 # np.random.uniform(-0.1,0.1,1)

		self.vel_publisher.publish(self.vel_msg)


if __name__ == "__main__":
	rospy.init_node("control_py")
	control = control_py()
	while True :
		control.controller()
		rospy.sleep(1.)
	
	rospy.spin()