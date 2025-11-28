"""
Shared constants for ArUco marker detection and identification.
Marker IDs are configured via launch files.
"""

from shared_things.packing import Box, Bin

# Marker IDs (defaults, can be overridden via launch file)
BOXES = {
    1: Box(name='mmm bread', length=1.0, width=1.0, height=1.0, id=1),
    2: Box(name='aaa apple', length=1.0, width=1.0, height=1.0, id=2),
    3: Box(name='ooo orange', length=1.0, width=1.0, height=1.0, id=3),
    4: Box(name='cheezzz', length=1.0, width=1.0, height=1.0, id=4),
    5: Box(name='bobo', length=1.0, width=1.0, height=1.0, id=5),
    6: Box(name='kiki', length=1.0, width=1.0, height=1.0, id=6),
}

BINS = {
    # 6: Bin(name='bag', length=10.0, width=10.0, height=1.0, id=6),
    7: Bin(name='box', length=10.0, width=10.0, height=1.0, id=7),
}

BOX_MARKER_IDS = BOXES.keys()
BIN_MARKER_IDS = BINS.keys()

# Consolidated readable descriptions for all markers. Import this from
# other packages to get a human-friendly name for each marker id.
BOX_ID_DESCRIPTIONS = {box.id: box.name for box in BOXES.values()} # id: name
BIN_ID_DESCRIPTIONS = {bin.id: bin.name for bin in BINS.values()}
MARKER_ID_DESCRIPTIONS = {**BOX_ID_DESCRIPTIONS, **BIN_ID_DESCRIPTIONS}

MARKER_OBJECTS = {**BOXES, **BINS}

# Marker sizes in meters
BOX_MARKER_SIZE = 0.15
BIN_MARKER_SIZE = 0.20
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

def get_marker_size(marker_id):
    if marker_id in BOXES:
        return BOX_MARKER_SIZE
    elif marker_id in BINS:
        return BIN_MARKER_SIZE
    else:
        return DEFAULT_MARKER_SIZE