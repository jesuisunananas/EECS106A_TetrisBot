import numpy as np
from box import Box, Bin

bin = Bin(2,4,5)

'''
Bin(2,4,x) height map is initially:
[[0, 0, 0, 0],
 [0, 0, 0, 0]]

when we add a Box(2, 1, 3) (from left-top point 0,0) height map becomes:
[[3, 3, 0, 0],
 [0, 0, 0, 0]]

2 questions we ask: where do we want to put it and is it legal?
'''
def add_box(x, y, box):
    if not can_add_box(x, y, box):
        return False
    submatrix = bin.height_map[y:y+box.width, x:x+box.length]
    base_height = submatrix.max()
    top_height = base_height + box.height
    bin.height_map[y:y + box.width, x:x + box.length] = top_height
    return True
    

def can_add_box(x, y, box):
    if x < 0: return False
    if y< 0: return False
    if x + box.length > bin.height_map.shape[1]: return False
    if y + box.width > bin.height_map.shape[0]: return False
    submatrix = bin.height_map[y:y+box.width, x:x+box.length]
    if np.any((submatrix + box.height) > bin.height): return False
    base_height = submatrix.max()
    if (((np.count_nonzero(submatrix == base_height)) / submatrix.size) * 100) < 50: return False
    return True

def all_pos_for_box(box):
    hmap = bin.height_map
    rows, cols = hmap.shape
    positions = []

    for y in range(rows - box.width + 1):
        for x in range(cols - box.length + 1):
            if can_add_box(x, y, box):
                # Compute base height (for tie breaking)
                submatrix = hmap[y:y + box.width, x:x + box.length]
                z = submatrix.max()
                positions.append((x, y, z))

    return positions

def place_box_with_rule(box):
    """
    Choose candidate with:
      minimal z, then minimal y, then minimal x
    and place box there.
    """
    candidates = all_pos_for_box(box)
    if not candidates:
        return None

    best_x, best_y, best_z = min(candidates, key=lambda p: (p[2], p[1], p[0]))
    add_box(best_x, best_y, box)
    return best_x, best_y, best_z

b = Box(4, 1, 5)
print("Initial:\n", bin.height_map)
placement = place_box_with_rule(b)
print("Placement:", placement)
print("After:\n", bin.height_map)