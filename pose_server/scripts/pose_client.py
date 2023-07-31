#!/usr/bin/env python3

import rospy
from pose_msg.srv import pose

def send_pose(pose_number):
    rospy.wait_for_service('pose')
    try:
        
        pose_service = rospy.ServiceProxy('pose', pose)
        response = pose_service(pose_number)
        return response.result
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: " + str(e))
        return None

if __name__ == '__main__':
    rospy.init_node('pose_client')
    pose_number = 1  # Replace this with the desired pose number (1 to 6)
    result = send_pose(pose_number)
    if result is not None:
        rospy.loginfo("Response: " + result)
