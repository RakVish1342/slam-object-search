#!/usr/bin/env python

import rospy 

# from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection


# rospy.init_node("assemble_scans_to_cloud")
# rospy.init_node("laser2pc")
# rospy.wait_for_service("assemble_scans2")
# assemble_scans = rospy.ServiceProxy('assemble_scans2', AssembleScans2)
# pub = rospy.Publisher ("/laser_pointcloud", PointCloud2, queue_size=1)

# r = rospy.Rate (1)

# while (True):
#     try:
#         resp = assemble_scans(rospy.Time(0,0), rospy.get_rostime())
#         print "Got cloud with %u points" % len(resp.cloud.data)
#         pub.publish (resp.cloud)

#     except rospy.ServiceException, e:
#         print "Service call failed: %s"%e

#     r.sleep()

class laser2PC():
	"""docstringfor """
	def __init__(self,):
		self.laserProj = LaserProjection()
		self.pcPub = rospy.Publisher("laserPointCloud",pc2,queue_size=1)
		self.laserSub = rospy.Subscriber("scan",LaserScan,self.laserCallback)

	def laserCallback(self,data):

		cloud_out = self.laserProj.projectLaser(data)

		self.pcPub.publish(cloud_out)

if __name__ == "__main__":
	rospy.init_node("laser2pc")
	l2pc = laser2PC()
	rospy.spin()