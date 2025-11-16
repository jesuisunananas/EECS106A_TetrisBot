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
    bin.height_map[y:y+box.width, x:x+box.length] = box.height
    return True
    

def can_add_box(x, y, box):
    if x < 0: return False
    if y< 0: return False
    if x + box.length > bin.height_map.shape[1]: return False
    if y + box.width > bin.height_map.shape[0]: return False
    if box.height + bin.height_map[x][y] > bin.height: return False
    return True

b = Box(4,1,5)
print(bin.height_map)
add_box(0,0,b)
print(bin.height_map)