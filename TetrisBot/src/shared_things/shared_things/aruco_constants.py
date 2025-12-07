"""
Shared constants for ArUco marker detection and identification.
Marker IDs are configured via launch files.
"""

from shared_things.packing import *
import cv2
import numpy as np

# Marker IDs (defaults, can be overridden via launch file)
BOXES = {
    1: Box(name='cube', length=0.08, width=0.08, height=0.08, id=1),
    0: Box(name='rectangle', length=0.06, width=0.06, height=0.1, id=0),
    # 3: Box(name='ooo orange', length=1.0, width=1.0, height=1.0, id=3),
    # 4: Box(name='cheezzz', length=1.0, width=1.0, height=1.0, id=4),
    # 5: Box(name='bobo', length=1.0, width=1.0, height=1.0, id=5),
    # 6: Box(name='kiki', length=1.0, width=1.0, height=1.0, id=6),
}

BINS = {
    # 6: Bin(name='bag', length=10.0, width=10.0, height=1.0, id=6),
    100: Bin(name='bin', length=10, width=10, height=1, id=100),
}

BOX_MARKER_IDS = BOXES.keys()
BIN_MARKER_IDS = BINS.keys()

# Consolidated readable descriptions for all markers. Import this from
# other packages to get a human-friendly name for each marker id.
BOX_ID_DESCRIPTIONS = {box.id: box.name for box in BOXES.values()} # id: name
BIN_ID_DESCRIPTIONS = {bin.id: bin.name for bin in BINS.values()}

# FOR COLLISION OBJECTS:
TABLE_IDS = [50, 51, 52, 53]
# table = Bundle(name='table', length=5.07, width= 5.08, height=0.02, id=TABLE_IDS)

table = Bundle(name='table', length= 5.0, width= 5.0, height=0.02, id=TABLE_IDS)
BUNDLES = {id: table for id in TABLE_IDS}

# Joint dictionaries: 
MARKER_ID_DESCRIPTIONS = {**BOX_ID_DESCRIPTIONS, **BIN_ID_DESCRIPTIONS}
MARKER_OBJECTS = {**BOXES, **BINS, **BUNDLES}

# Marker sizes in meters
BOX_MARKER_SIZE = 0.05
BIN_MARKER_SIZE = 0.05
DEFAULT_MARKER_SIZE = 0.15 # NOTE: Not sure why marker_size is relevant but 
                           # gonna do this so when id not in marker_size_map, 
                           # it doesnt break.


# Utility functions
def get_object_by_id(marker_id):
    """get"""
    return MARKER_OBJECTS.get(marker_id)

def is_box(marker_id):
    """is this ID a box?"""
    return marker_id in BOXES

def is_bin(marker_id):
    """is this ID a bin?"""
    return marker_id in BINS

def is_table(marker_id):
    """is this marker used to define the table?"""
    return marker_id in TABLE_IDS

def get_marker_size(marker_id):
    if marker_id in BOXES:
        return BOX_MARKER_SIZE
    elif marker_id in BINS:
        return BIN_MARKER_SIZE
    elif marker_id in TABLE_IDS:
        return BIN_MARKER_SIZE
    else:
        return DEFAULT_MARKER_SIZE
    
def custom_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    Simplified version that matches the original function's output format
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    
    rvecs = []
    tvecs = []
    trash = []
    
    for i, c in enumerate(corners):
        # Ensure corners are in the right format (4, 2)
        corner_points = c.reshape(-1, 2) if len(c.shape) == 3 else c
        
        success, rvec, tvec = cv2.solvePnP(marker_points, corner_points, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        
        # Reshape to (1, 3) to match original format
        rvecs.append(rvec.reshape(1, 3))
        tvecs.append(tvec.reshape(1, 3))
        trash.append(success)
    
    # Stack into arrays of shape (N, 1, 3)
    rvecs = np.array(rvecs) if rvecs else np.empty((0, 1, 3))
    tvecs = np.array(tvecs) if tvecs else np.empty((0, 1, 3))
    
    return rvecs, tvecs, trash