import numpy as np # pyright: ignore[reportMissingImports]
from box import Box, Bin

bin = Bin(4,4,5)

'''
Bin(2,4,x) height map is initially:
[[0, 0, 0, 0],
 [0, 0, 0, 0]]

when we add a Box(2, 1, 3) (from left-top point 0,0) height map becomes:
[[3, 3, 0, 0],
 [0, 0, 0, 0]]

2 questions we ask: where do we want to put it and is it legal?
'''
def add_box(x, y, box: Box, b: Bin):
    if not can_add_box(x, y, box, b):
        return False
    submatrix = b.height_map[y:y+box.width, x:x+box.length]
    base_height = submatrix.max()
    top_height = base_height + box.height
    b.height_map[y:y + box.width, x:x + box.length] = top_height
    b.boxes[box.name] = {
        "box": box,
        "x": x,
        "y": y,
        "z": base_height
    }
    update_access_priority(b)
    return True

def can_add_box(x, y, box: Box, b: Bin):
    if x < 0: return False
    if y< 0: return False
    if x + box.length > b.height_map.shape[1]: return False
    if y + box.width > b.height_map.shape[0]: return False
    submatrix = b.height_map[y:y+box.width, x:x+box.length]
    if np.any((submatrix + box.height) > b.height): return False
    base_height = submatrix.max()
    support_ratio = np.count_nonzero(submatrix == base_height) / submatrix.size
    if support_ratio < 0.5:
        return False
    return True

def update_access_priority(b: Bin):
    entries = list(b.boxes.items())
    entries.sort(key=lambda item: item[1]["box"].fragility)
    b.priority_list = [i for i, _ in entries]

def all_pos_for_box(box: Box, b: Bin):
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

def place_box_with_rule(box: Box, b: Bin):
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

def compute_compactness(b: Bin):
    max_height = np.max(b.height_map)
    bounding_volume = max_height * b.length * b.width
    object_volume = 0
    for i in b.boxes.values():
        object_volume += i["box"].volume
    C_i = object_volume / bounding_volume
    return C_i

def compute_pyramid(b: Bin):
    object_volume = 0
    mask = np.zeros_like(b.height_map, dtype=bool)
    for entry in b.boxes.values():
        box = entry["box"]
        x = entry["x"]
        y = entry["y"]
        mask[y:y + box.width, x:x + box.length] = True
    for i in b.boxes.values():
        object_volume += i["box"].volume
    region_volume = float(b.height_map[mask].sum())
    if region_volume == 0:
        return 0.0
    return object_volume / region_volume

def compute_access_cost(b: Bin):
    a_c = 0
    for i, box in enumerate(b.priority_list):
        curr_box = b.boxes[box]
        p_i = len(b.priority_list) - i
        z_i = b.height - (curr_box["box"].height + curr_box["z"])
        a_c += z_i * p_i
    return a_c

def compute_fragility_penalty(b: Bin):
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