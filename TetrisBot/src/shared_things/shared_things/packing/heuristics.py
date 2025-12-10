import numpy as np # pyright: ignore[reportMissingImports]
from .box import Box, Bin

bin = Bin(4,4,5, id=7)

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
    submatrix = b.height_map[y:y+box.grid_width(b.resolution), x:x+box.grid_length(b.resolution)]
    base_height = submatrix.max()
    top_height = base_height + box.grid_height(b.resolution)
    b.height_map[y:y + box.grid_width(b.resolution), x:x + box.grid_length(b.resolution)] = top_height
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
    if x + box.grid_length(b.resolution) > b.height_map.shape[1]: return False
    if y + box.grid_width(b.resolution) > b.height_map.shape[0]: return False

    submatrix = b.height_map[y:y+box.grid_width(b.resolution), x:x+box.grid_length(b.resolution)]
    if np.any((submatrix + box.grid_height(b.resolution)) > b.height): return False
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
    # width = int(round(box.width / b.resolution))
    # length = int(round(box.length / b.resolution))
    print(f'box grid width: {box.grid_width(b.resolution)}')

    for y in range(rows - box.grid_width(b.resolution) + 1):
        for x in range(cols - box.grid_length(b.resolution) + 1):
            if can_add_box(x, y, box, b):
                # Compute base height (for tie breaking)
                submatrix = hmap[y:y + box.grid_width(b.resolution), x:x + box.grid_length(b.resolution)]
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
    if max_height == 0.0:
        return 0.0
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
        mask[y:y + box.grid_width(b.resolution), x:x + box.grid_length(b.resolution)] = True
        object_volume += entry["box"].volume
    region_volume = float(b.height_map[mask].sum())
    if region_volume == 0:
        return 0.0
    return object_volume / region_volume

def compute_access_cost(b: Bin):
    if not b.priority_list:
        return 0.0
    N = len(b.priority_list)
    H = float(b.height)
    a_c = 0
    for i, box in enumerate(b.priority_list):
        curr_box = b.boxes[box]
        p_i = N - i
        z_i = H - (curr_box["box"].height + curr_box["z"])
        z_norm = max(0.0, min(1.0, z_i / H))
        fragility = float(curr_box["box"].fragility)
        frag_weight = 1.0 #+ 2.0 * (fragility - 1.0)
        a_c += frag_weight * z_norm * p_i
    return a_c / N
    
def footprint_overlap(b1: Box, b2: Box, b: Bin):
    x_j = b.boxes[b1.name]["x"]
    y_j = b.boxes[b1.name]["y"]
    x_k = b.boxes[b2.name]["x"]
    y_k = b.boxes[b2.name]["y"] 
    x_overlap = max(0, min(x_j + b1.length, x_k + b2.length) - max(x_j, x_k))
    y_overlap = max(0, min(y_j + b1.grid_width(b.resolution), y_k + b2.grid_width(b.resolution)) - max(y_j, y_k))
    return x_overlap * y_overlap

# check if b2 is stacked on b1
def vertical_stacking(b1: Box, b2: Box, b: Bin):
    z_top = b1.grid_height(b.resolution) + b.boxes[b1.name]["z"]
    z_base = b.boxes[b2.name]["z"]
    return z_base >= z_top

def weight_on_box(lower: Box, upper: Box, b: Bin):
    #densityconstant of object p*volume = weight can experiment with p
    p = 1.0
    area_overlap = footprint_overlap(lower, upper, b)
    if area_overlap == 0.0:
        return 0.0
    area_upper = upper.length * upper.width
    fraction_on_lower = area_overlap / area_upper
    return fraction_on_lower * p * upper.volume


def compute_fragility_penalty(b: Bin,
                              base_scaling,
                              heavy_factor,
                              fragile_quantile,
                              alpha) -> float:
    """
    Fragility penalty:
      - For each box j, compute load_on_box from boxes stacked above.
      - Define "very fragile" as having fragility <= quantile of all fragilities.
      - For very fragile boxes, apply a much larger penalty if something sits on top.
    """

    if not b.boxes:
        return 0.0

    # --- 1. Collect fragilities and compute "very fragile" threshold ---
    frag_list = np.array([entry["box"].fragility for entry in b.boxes.values()], dtype=float)
    # e.g. bottom 25% are "very fragile"
    very_fragile_thresh = np.quantile(frag_list, fragile_quantile)

    penalty = 0.0

    # --- 2. Loop over each "lower" box j ---
    for _, j_entry in b.boxes.items():
        j_box = j_entry["box"]
        frag_j = float(j_box.fragility)

        # compute how much weight is on j_box
        load_on_box = 0.0
        for _, k_entry in b.boxes.items():
            k_box = k_entry["box"]
            if not vertical_stacking(j_box, k_box, b):
                continue
            load_on_box += weight_on_box(j_box, k_box, b)

        # "capacity" based on fragility & volume
        capacity = alpha * frag_j * float(j_box.volume)

        overload = max(0.0, load_on_box - capacity)
        if overload <= 0.0:
            # no overload => no penalty for this box
            continue

        # --- 3. Heavier penalty if j is very fragile compared to others ---
        if frag_j <= very_fragile_thresh:
            # Very fragile relative to set: big penalty
            scale = base_scaling * heavy_factor
        else:
            # Normal box
            scale = base_scaling

        penalty += scale * overload

    return float(penalty)

        


# b = Box(1, 1, 5, id=1)
# b1 = Box(1,2,2, id=2)
# b2 = Box(2,3,1, id=3)
# b3 = Box(2,2,2, id=4)
# print("Initial:\n", bin.height_map)
# placement = place_box_with_rule(b, bin)
# print("Placement:", placement)
# placement = place_box_with_rule(b1, bin)
# print("Placement:", placement)
# placement = place_box_with_rule(b2, bin)
# print("Placement:", placement)
# placement = place_box_with_rule(b3, bin)
# print("Placement:", placement)
# print("After:\n", bin.height_map)