#!/usr/bin/env python
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
import rospy
from std_msgs.msg import String
import triad_openvr
import time
import sys
import tf
from tf.transformations import quaternion_from_euler
from tf.transformations import quaternion_multiply
import numpy as np
import math
import pdb
import openvr
def tracker():
    #rospy.init_node('vive_tracker_frame')
    #broadcaster = { }
    #publisher = { }
    #listener = tf.TransformListener()
    #rate = rospy.Rate(20) # 20hz]
    deviceCount = 0
    track_name = ""
    
    try:
      v = triad_openvr.triad_openvr()
    except Exception as ex:
      if (type(ex).__name__ == 'OpenVRError' and ex.args[0] == 'VRInitError_Init_HmdNotFoundPresenceFailed (error number 126)'):
        print('Cannot find the tracker.')
        print('Is SteamVR running?')
        print('Is the Vive Tracker turned on, connected, and paired with SteamVR?')
        print('Are the Lighthouse Base Stations powered and in view of the Tracker?\n\n')
      else:
        template = "An exception of type {0} occurred. Arguments:\n{1!r}"
        message = template.format(type(ex).__name__, ex.args)
        print message
      #print(ex.args)
      quit()

    v.print_discovered_objects()
    
    for deviceName in v.devices:
        if "LHR" in v.devices[deviceName].get_serial():
            track_name = deviceName
            
    while(True):
#        pose = v.vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,openvr.k_unMaxTrackedDeviceCount)
        [x,y,z,roll,pitch,yaw] = v.devices[track_name].get_pose_euler()
        print('X: %f, Y: %f, Z: %f, Roll: %f, Pitch: %f, Yaw: %f' % (x, y, z, roll, pitch, yaw))
        #print pose[v.devices[track_name].index].mDeviceToAbsoluteTracking[0][3], pose[v.devices[track_name].index].mDeviceToAbsoluteTracking[1][3], pose[v.devices[track_name].index].mDeviceToAbsoluteTracking[2][3]
if __name__ == '__main__':
    tracker()
