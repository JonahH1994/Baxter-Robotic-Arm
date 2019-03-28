#!/usr/bin/env python  
import roslib
roslib.load_manifest('octomap_server')
import rospy
 
import tf
import turtlesim.msg
 
if __name__ == '__main__':
  rospy.init_node('kinect_baxter_broadcaster')

  br = tf.TransformBroadcaster()
  br1 = tf.TransformBroadcaster()
  br2 = tf.TransformBroadcaster()
  br3 = tf.TransformBroadcaster()

  rate = rospy.Rate(10.0)

  while not rospy.is_shutdown():
    br.sendTransform((0, 0, 1.104),
                      tf.transformations.quaternion_from_euler(0, 0, 0),
                      rospy.Time.now(),
                      "base",
                      "world")
    br1.sendTransform((0,0,0),
                      tf.transformations.quaternion_from_euler(0,0,0),
                      rospy.Time.now(),
                      "torso",
                      "base")

    br2.sendTransform((0.06, 0, 0.686),
			#(0, 0, 0.114445, 0.9934),
                      tf.transformations.quaternion_from_euler(0,0,0),
                      rospy.Time.now(),
                      "head",
                      "torso")

    br3.sendTransform((0.05, 0, 0.23),
                      tf.transformations.quaternion_from_euler(4.2, 0, 0),
                      rospy.Time.now(),
                      "camera_link",
                      "head")

    rate.sleep()
