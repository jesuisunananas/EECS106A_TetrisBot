"""
Shared constants for ArUco marker detection and identification.
Marker IDs are configured via launch files.
"""

# Marker IDs (defaults, can be overridden via launch file)
BOX_ID_DESCRIPTIONS = {
    1: 'mmm bread',
    2: 'aaa apple',
    3: 'ooo orange',
    4: 'cheezzz',
    5: 'baby',
    11: 'gun',
}

BIN_ID_DESCRIPTIONS = {
    6: 'bag',
    7: 'box',
}

BOX_MARKER_IDS = BOX_ID_DESCRIPTIONS.keys()
BIN_MARKER_IDS = BIN_ID_DESCRIPTIONS.keys()

# Consolidated readable descriptions for all markers. Import this from
# other packages to get a human-friendly name for each marker id.
MARKER_ID_DESCRIPTIONS = {**BOX_ID_DESCRIPTIONS, **BIN_ID_DESCRIPTIONS}

# Marker sizes in meters
BOX_MARKER_SIZE = 0.15
BIN_MARKER_SIZE = 0.20
DEFAULT_MARKER_SIZE = 0.15 # NOTE: Not sure why marker_size is relevant but 
                           # gonna do this so when id not in marker_size_map, 
                           # it doesnt break.
