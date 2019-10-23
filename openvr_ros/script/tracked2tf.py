#!/usr/bin/env python

import rospy
import tf_conversions
import tf2_ros

from openvr_ros.msg import TrackedDevicePose
from openvr_ros.msg import TrackedDeviceClass, TrackedDeviceRole, TrackedDeviceResult

from geometry_msgs.msg import TransformStamped


class Tracked2TF:
    def __init__(self, pose_topic):
        rospy.Subscriber(pose_topic, TrackedDevicePose, self.callback, queue_size=1200)
        self.br = tf2_ros.TransformBroadcaster()

        # We assign the roles ourselves
        self.role_names = ["left", "right"]
        self.class_names = ["invalid", "hmd", "controller", "tracker", "reference", "display"]
        
        # Keeps track of a persistent name for a device_id
        self.name = {}
        # The "key" of this dict is the class number. This dict keeps number of devices in each class.
        self.count = {0:0, 1:0, 2:0, 3:0, 4:0, 5:0}
    
    def register(self, ID, Class):
        if not ID in self.name:
            self.count[Class] += 1
            if Class == 1:
                self.name[ID] = self.class_names[Class]
            else:
                self.name[ID] = self.class_names[Class] + "." + self.role_names[self.count[Class]%2]
        return self.name[ID]

    def callback(self, data):
        # 0. Register the role and class of the object
        # 1. Make the name of the frame from its class and its role.
        # 2. If TrackingResult, PoseIsValid, and DeviceIsConnected are in good status then pblish the device position.

        # "data" type: TrackedDevicePose()
        tf_msg = TransformStamped()
        
        frame_child = self.register(data.device_header.ID, data.device_header.Class)

        tf_msg.header.stamp = data.header.stamp
        tf_msg.header.frame_id = "tracker_link"
        tf_msg.child_frame_id = frame_child
        tf_msg.transform.translation = data.pose.position
        tf_msg.transform.rotation = data.pose.orientation
        self.br.sendTransform(tf_msg)

if __name__=="__main__":
    rospy.init_node('tracked2tf', anonymous=True)
    pose_topic = rospy.get_param("~pose_topic")
    Tracked2TF(pose_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS Image feature detector module")


