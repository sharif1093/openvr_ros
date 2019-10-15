#ifndef UTILITY_H_
#define UTILITY_H_

#include<string>  
#include <openvr/openvr.h>
#include "ros/ros.h"
#include "openvr_ros/TrackedDevicePose.h"
// // ENUM TYPES
// #include "openvr_ros/TrackedDeviceClass.h"
// #include "openvr_ros/TrackedDeviceResult.h"
// #include "openvr_ros/TrackedDeviceRole.h"


namespace openvr
{

// Initialization of the VR System
inline vr::IVRSystem* initialize()
{
    vr::EVRInitError eError = vr::VRInitError_None;
    vr::IVRSystem* vr_pointer = NULL;
    vr_pointer = vr::VR_Init(&eError, vr::VRApplication_Background);
    if (eError != vr::VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n", 
            vr::VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }
    return vr_pointer;
}

inline void shutdown(vr::IVRSystem* vr_pointer)
{
	if (vr_pointer != NULL)
	{
		vr::VR_Shutdown(); 
		vr_pointer = NULL;
	}
}

inline void PollEvents(vr::IVRSystem* vr_pointer)
{
    vr::VREvent_t event;
    static const char* classNames[] = {"Invalid Device", "HMD", "Controller", "Tracker", "Tracking Reference", "Display Redirect"};
    static const char* roleNames[]  = {"invalid", "left", "right", "opt out", "treadmill"};

    if(vr_pointer->PollNextEvent(&event, sizeof(event)))
	{
        vr::TrackedDeviceIndex_t id = event.trackedDeviceIndex;
        uint32_t button = event.data.controller.button;
        vr::ETrackedDeviceClass deviceClass = vr_pointer->GetTrackedDeviceClass(id);
        vr::ETrackedControllerRole deviceRole = vr_pointer->GetControllerRoleForTrackedDeviceIndex(id);

        switch(event.eventType)
        {
            case vr::VREvent_TrackedDeviceActivated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) attached.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]);
                break;
            case vr::VREvent_TrackedDeviceDeactivated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) detached.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]); 
                break;
            case vr::VREvent_TrackedDeviceUpdated:
                ROS_WARN("EVENT (OpenVR) %s (id=%d, role=%s) updated.", classNames[(int)deviceClass], id, roleNames[(int)deviceRole]);
                break;
            case vr::VREvent_EnterStandbyMode:
                ROS_WARN("EVENT (OpenVR) Entered Standby mode.");
                break;
            case vr::VREvent_LeaveStandbyMode:
                ROS_WARN("EVENT (OpenVR) Leave Standby mode.");
                break;

            default:
                // printf("EVENT--(OpenVR) Event: %d\n", event.eventType); 
                break;
        }
	}
}


//-----------------------------------------------------------------------------
// Purpose: Calculates quaternion (qw,qx,qy,qz) representing the rotation
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
// from: https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
//-----------------------------------------------------------------------------

vr::HmdQuaternion_t GetRotation(vr::HmdMatrix34_t matrix) {
    vr::HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
    return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
// from: https://www.codeproject.com/Articles/1171122/How-to-Get-Raw-Positional-Data-from-HTC-Vive
//-----------------------------------------------------------------------------

vr::HmdVector3_t GetPosition(vr::HmdMatrix34_t matrix) {
    vr::HmdVector3_t vector;

    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];

    return vector;
}

inline void PublishTrackedDevicePose(vr::IVRSystem* vr_pointer,
                                     const ros::Publisher &publisher,
                                     vr::TrackedDeviceIndex_t id,
                                     const vr::TrackedDevicePose_t &trackedDevicePose)
{   
    // Do not waste time, just record this moment's timestamp.
    openvr_ros::TrackedDevicePose msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "";

    vr::ETrackedDeviceClass trackedDeviceClass   = vr_pointer->GetTrackedDeviceClass(id);
    vr::ETrackedControllerRole trackedDeviceRole = vr_pointer->GetControllerRoleForTrackedDeviceIndex(id);

    msg.device_header.ID = (uint16_t) id;
    msg.device_header.Class = (uint8_t) trackedDeviceClass;
    msg.device_header.Role  = (uint8_t) trackedDeviceRole;
    msg.device_header.TrackingResult = (uint8_t) trackedDevicePose.eTrackingResult;
    msg.device_header.PoseIsValid = (bool) trackedDevicePose.bPoseIsValid;
    msg.device_header.DeviceIsConnected = (bool) trackedDevicePose.bDeviceIsConnected;

    vr::HmdVector3_t position;
    position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
    msg.pose.position.x = position.v[0];
    msg.pose.position.y = position.v[1];
    msg.pose.position.z = position.v[2];
    
    vr::HmdQuaternion_t quaternion;
    quaternion = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
    msg.pose.orientation.x = quaternion.x;
    msg.pose.orientation.y = quaternion.y;
    msg.pose.orientation.z = quaternion.z;
    msg.pose.orientation.w = quaternion.w;

    vr::HmdVector3_t velocity;
    velocity = trackedDevicePose.vVelocity;
    msg.velocity.linear.x = velocity.v[0];
    msg.velocity.linear.y = velocity.v[1];
    msg.velocity.linear.z = velocity.v[2];

    vr::HmdVector3_t angular;
    angular = trackedDevicePose.vAngularVelocity;
    msg.velocity.angular.x = angular.v[0];
    msg.velocity.angular.y = angular.v[1];
    msg.velocity.angular.z = angular.v[2];

    publisher.publish(msg);
}

inline void PollPoses(vr::IVRSystem* vr_pointer, const ros::Publisher &publisher)
{
    for (vr::TrackedDeviceIndex_t id = 0; id < vr::k_unMaxTrackedDeviceCount; id++)
    {
        if (!vr_pointer->IsTrackedDeviceConnected(id))
            continue;

        vr::ETrackedDeviceClass trackedDeviceClass = vr_pointer->GetTrackedDeviceClass(id);
        
        vr::TrackedDevicePose_t trackedDevicePose;
        vr::VRControllerState_t controllerState;
        
        switch (trackedDeviceClass)
        {
            case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
                // printf("Headset\n");
                vr_pointer->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
                // printf("Controller: %d\n", id);
                vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, id, &controllerState, sizeof(controllerState), &trackedDevicePose);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;
            case vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker:
                // printf("Tracker: %d\n", id);
                vr_pointer->GetControllerStateWithPose(vr::TrackingUniverseStanding, id, &controllerState, sizeof(controllerState), &trackedDevicePose);
                PublishTrackedDevicePose(vr_pointer, publisher, id, trackedDevicePose);
                break;
            default:    
                break;
        }
    }
}

} // namespace openvr
#endif // UTILITY_H_
