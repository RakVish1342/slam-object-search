#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')
import rospy

import tf
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
# import turtlesim.msg

def handle_turtle_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.data[0], msg.data[1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.data[2]),
                     rospy.Time.now(),
                     "base_scan",
                     "start_frame")

    br0 = tf.TransformBroadcaster()
    br0.sendTransform((msg.data[3], msg.data[4], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "marker_0",
                     "start_frame")

    br1 = tf.TransformBroadcaster()
    br1.sendTransform((msg.data[5], msg.data[6], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "marker_1",
                     "start_frame")


    br2 = tf.TransformBroadcaster()
    br2.sendTransform((msg.data[7], msg.data[8], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "marker_2",
                     "start_frame")

    br3 = tf.TransformBroadcaster()
    br3.sendTransform((msg.data[9], msg.data[10], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "marker_3",
                     "start_frame")

    br4 = tf.TransformBroadcaster()
    br4.sendTransform((msg.data[11], msg.data[12], 0),
                     tf.transformations.quaternion_from_euler(0, 0, 0),
                     rospy.Time.now(),
                     "marker_4",
                     "start_frame")


def handle_turtle_pose_odom(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     tf.transformations.quaternion_from_euler(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z),
                     # tf.transformations.quaternion_from_euler(0,0,0),
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

    # rospy.Subscriber('/odom',
    #                  Odometry,
    #                  handle_turtle_pose_odom)

    rospy.spin()

