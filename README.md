# Lab2_ME445
Lab 2 files 


# Global variable to track suction status
gripper_status = 0  # 0 = Not Holding, 1 = Holding

def gripper_callback(msg):
    """
    Callback function to check if the suction cup is holding a block.
    """
    global gripper_status
    gripper_status = msg.Digital_Input_0  # Update status based on ROS message

    if gripper_status == 1:
        rospy.loginfo("Gripper is successfully holding a block.")
    else:
        rospy.logwarn("Gripper is NOT holding anything!")
