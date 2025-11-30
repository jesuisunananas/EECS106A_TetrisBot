import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from shared_things import *
import rclpy
from tf2_ros import TransformException

import numpy as np
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

def place_table(table_poses):

    
    # --- Configuration ---
    TABLE_DIMS = [2.0, 2.0, 0.05] # 2m x 2m x 5cm
    
    normals = np.array([])
    positions = np.array([])
    
    for pose in table_poses:
        rx = pose.orientation.x
        ry = pose.orientation.y
        rz = pose.orientation.z
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        np.append(normals, np.array([rx, ry, rz]), axis=0)
        np.append(positions, np.array([x, y, z]), axis=0)
        
    average_normal = np.mean(normals, axis=0)
    average_pos = np.mean(positions, axis=0)
    
    if dist > STABILITY_DIST:
        # Unstable / Moved: Reset timer and reference
        state['ref_pos'] = avg_pos
        state['start_time'] = current_time
        return None
    
    # 6. Check Duration
    elapsed = (current_time - state['start_time']).nanoseconds / 1e9
    if elapsed < STABILITY_DUR:
        return None # Stable, but not long enough yet

    # --- SUCCESS: Create Collision Object ---
    state['frozen'] = True 

    # Calculate Collision Pose
    # Z is shifted down by half thickness so the top surface aligns with tags
    box_pose = Pose()
    box_pose.position.x = avg_pos[0]
    box_pose.position.y = avg_pos[1]
    box_pose.position.z = avg_pos[2] - (TABLE_DIMS[2] / 2.0) 
    
    # Use the orientation of the first tag for alignment
    # (Averaging quaternions is complex; this is usually sufficient for a flat table)
    box_pose.orientation = table_poses[0].orientation

    # Create Primitive
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = TABLE_DIMS

    # Create Object
    coll_obj = CollisionObject()
    coll_obj.header.frame_id = target_frame
    coll_obj.id = "work_table_surface"
    coll_obj.primitives = [box]
    coll_obj.primitive_poses = [box_pose]
    coll_obj.operation = CollisionObject.ADD
    
    return coll_obj