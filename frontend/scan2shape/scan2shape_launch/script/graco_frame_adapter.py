#!/usr/bin/env python3
"""
Adapts GrAco frame names to what SlideSLAM expects.

GrAco:     base_enu -> gnss
SlideSLAM: odom     -> body

Does two things:
  1. Republishes /Odometry with corrected frame_id/child_frame_id
  2. Broadcasts the dynamic odom->body TF so process_cloud_node can look it up
"""
import rospy
import tf
from nav_msgs.msg import Odometry


class FrameAdapter:
    def __init__(self):
        robot_name = rospy.get_param('~robot_name', '')
        prefix = robot_name + '/' if robot_name else ''
        self.odom_frame = prefix + 'odom'
        self.body_frame = prefix + 'body'
        odom_in = '/' + prefix + 'Odometry'
        odom_out = '/' + prefix + 'Odometry_adapted'

        self.use_ros_time = rospy.get_param('~use_ros_time', False)
        self.broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher(odom_out, Odometry, queue_size=10)
        rospy.Subscriber(odom_in, Odometry, self.cb, queue_size=100)

    def cb(self, msg):
        t = msg.pose.pose.position
        q = msg.pose.pose.orientation

        stamp = rospy.Time.now() if self.use_ros_time else msg.header.stamp

        self.broadcaster.sendTransform(
            (t.x, t.y, t.z),
            (q.x, q.y, q.z, q.w),
            stamp,
            self.body_frame,
            self.odom_frame
        )

        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.body_frame
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('graco_frame_adapter')
    FrameAdapter()
    rospy.spin()
