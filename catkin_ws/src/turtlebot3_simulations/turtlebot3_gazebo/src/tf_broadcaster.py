#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')
import rospy

import tf
from std_msgs.msg import Float64MultiArray
# import turtlesim.msg

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.data[0], msg.data[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.data[2]),
                     rospy.Time.now(),
                     "base_scan",
                     "start_frame")

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    # turtlename = rospy.get_param('~turtle')
    # rospy.Subscriber('/%s/pose' % turtlename,
    #                  turtlesim.msg.Pose,
    #                  handle_turtle_pose,
    #                  turtlename)
    rospy.Subscriber('/turtle/states',
                     Float64MultiArray,
                     handle_turtle_pose)

    rospy.spin()

