import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from shared_things import *
import rclpy
from tf2_ros import TransformException

def average_table_pose(table, table_poses):
    """
    Calculates the average pose of a table based on a list of detected AR tag poses.
    """
    
    if not table_poses:
        return None

    # Use lists for collection (faster than np.append in a loop)
    positions = []
    quats = []

    for pose in table_poses:
        # Collect Position
        positions.append([pose.position.x, pose.position.y, pose.position.z])
        
        # Collect Orientation (Quaternion x,y,z,w)
        # Note: We must handle the sign ambiguity of quaternions (q == -q)
        # For simple table cases where tags are mostly aligned, 
        # standard averaging usually works if signs are consistent.
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        
        # Ensure quaternion consistency (flip if dot product with first is negative)
        if quats and np.dot(quats[0], q) < 0:
            q = [-val for val in q]
            
        quats.append(q)
        
    # Calculate Averages
    average_pos = np.mean(positions, axis=0)
    average_quat = np.mean(quats, axis=0)

    # CRITICAL: Normalize the quaternion!
    # A quaternion must have length 1 to be valid. Averaging destroys this length.
    norm = np.linalg.norm(average_quat)
    if norm == 0:
        average_quat = np.array([0, 0, 0, 1]) # Fallback to identity
    else:
        average_quat = average_quat / norm
    
    # Calculate Collision Pose
    box_pose = Pose()
    box_pose.position.x = average_pos[0]
    box_pose.position.y = average_pos[1]
    # Z is shifted down by half thickness so the top surface aligns with tags
    box_pose.position.z = average_pos[2] - (table.height / 2.0) 

    # Assign Valid Normalized Orientation
    box_pose.orientation.x = average_quat[0]
    box_pose.orientation.y = average_quat[1]
    box_pose.orientation.z = average_quat[2]
    box_pose.orientation.w = average_quat[3]

    return box_pose