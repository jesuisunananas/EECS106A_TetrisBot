import numpy as np
from box import Box, Bin

bin = Bin(4,4,5)
boxes = {}

'''
Bin(2,4,x) height map is initially:
[[0, 0, 0, 0],
 [0, 0, 0, 0]]

when we add a Box(2, 1, 3) (from left-top point 0,0) height map becomes:
[[3, 3, 0, 0],
 [0, 0, 0, 0]]

2 questions we ask: where do we want to put it and is it legal?
'''
def add_box(x, y, box, b):
    if not can_add_box(x, y, box, b):
        return False
    submatrix = b.height_map[y:y+box.width, x:x+box.length]
    base_height = submatrix.max()
    top_height = base_height + box.height
    b.height_map[y:y + box.width, x:x + box.length] = top_height
    b.boxes[box.name] = box.volume
    return True
    

def can_add_box(x, y, box, b):
    if x < 0: return False
    if y< 0: return False
    if x + box.length > b.height_map.shape[1]: return False
    if y + box.width > b.height_map.shape[0]: return False
    submatrix = b.height_map[y:y+box.width, x:x+box.length]
    if np.any((submatrix + box.height) > b.height): return False
    base_height = submatrix.max()
    if (((np.count_nonzero(submatrix == base_height)) / submatrix.size) * 100) < 50: return False
    return True

def all_pos_for_box(box, b):
    hmap = b.height_map
    rows, cols = hmap.shape
    positions = []

    for y in range(rows - box.width + 1):
        for x in range(cols - box.length + 1):
            if can_add_box(x, y, box, b):
                # Compute base height (for tie breaking)
                submatrix = hmap[y:y + box.width, x:x + box.length]
                z = submatrix.max()
                positions.append((x, y, z))

    return positions

def place_box_with_rule(box, b):
    """
    Choose candidate with:
      minimal z, then minimal y, then minimal x
    and place box there.
    """
    candidates = all_pos_for_box(box, b)
    if not candidates:
        return None

    best_x, best_y, best_z = min(candidates, key=lambda p: (p[2], p[1], p[0]))
    add_box(best_x, best_y, box, b)
    return best_x, best_y, best_z

def compute_compactness(bin):
    max_height = np.max(bin.height_map)
    bounding_volume = max_height * bin.length * bin.width
    object_volume = 0
    for i in bin.boxes:
        object.volume += i.volume
    C_i = object_volume / bounding_volume

def compute_pyramid():
    pass

def compute_access_cost():
    pass

def compute_fragility_penalty():
    pass

def compute_access_priority():
    pass

b = Box(1, 1, 5)
b1 = Box(1,2,2)
b2 = Box(2,3,1)
b3 = Box(2,2,2)
print("Initial:\n", bin.height_map)
placement = place_box_with_rule(b, bin)
print("Placement:", placement)
placement = place_box_with_rule(b1, bin)
print("Placement:", placement)
placement = place_box_with_rule(b2, bin)
print("Placement:", placement)
placement = place_box_with_rule(b3, bin)
print("Placement:", placement)
print("After:\n", bin.height_map)