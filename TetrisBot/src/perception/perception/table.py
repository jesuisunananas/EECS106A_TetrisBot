import numpy as np
from geometry_msgs.msg import Pose
from shared_things import *

def average_table_pose(table, table_poses: list[Pose]):
    """
    Calculates the average pose of a table based on a list of detected AR tag poses.
    """
    
    if not table_poses:
        return None

    x_sum = 0 
    y_sum = 0
    z_sum = 0
    
    qx_sum = 0
    qy_sum = 0
    qz_sum = 0
    qw_sum = 0

    for pose in table_poses:
        x_sum += pose.position.x
        y_sum += pose.position.y
        z_sum += pose.position.z
        
        qx_sum += pose.orientation.x
        qy_sum += pose.orientation.y
        qz_sum += pose.orientation.z
        qw_sum += pose.orientation.w
        
    # Calculate Averages
    pose_num = len(table_poses)
    average_pos = [x_sum / pose_num, y_sum / pose_num, z_sum / pose_num]
    average_quat = [qx_sum / pose_num, qy_sum / pose_num, qz_sum / pose_num, qw_sum / pose_num]

    # Normalise the quaternion
    norm = np.linalg.norm(average_quat)
    if norm == 0:
        average_quat = np.array([0, 0, 0, 1]) # Fallback to identity
    else:
        average_quat = average_quat / norm
    
    box_pose = Pose()
    box_pose.position.x = average_pos[0]
    box_pose.position.y = average_pos[1]
    box_pose.position.z = average_pos[2] - (table.height / 2.0)    
    # ^ Z is shifted down by half thickness so the top surface aligns with tags

    box_pose.orientation.x = average_quat[0]
    box_pose.orientation.y = average_quat[1]
    box_pose.orientation.z = average_quat[2]
    box_pose.orientation.w = average_quat[3]

    return box_pose