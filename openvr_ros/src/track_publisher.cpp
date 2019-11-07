#include "ros/ros.h"
#include "openvr_ros/utility.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "openvr_ros");
  ros::NodeHandle n("~");
  ros::Publisher tracked_device_pose_publisher = n.advertise<openvr_ros::TrackedDevicePose>("tracked_device_pose", 300);

  vr::IVRSystem* vr_pointer = NULL;
  vr_pointer = openvr::initialize();

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    openvr::PollEvents(vr_pointer);
    openvr::PollPoses(vr_pointer, tracked_device_pose_publisher);

    // chatter_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  openvr::shutdown(vr_pointer);
  return 0;
}
