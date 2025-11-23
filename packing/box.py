import numpy as np
from geometry_msgs.msg import PoseStamped

class Box:
    def __init__(self, name, length, width, height, id, pose: PoseStamped = None, fragility=1.0):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.volume = self.length * self.width * self.height
        self.fragility = fragility
        
        # pose and ID from Kevin
        self.id = id
        self.pose = pose

    """Kevin: these are for updating the poses for the box"""
    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose: PoseStamped):
        self._pose = pose
        
    """Kevin: these are for updating the ids for the box"""
    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        if id < 0:
            raise ValueError("I don't think id's can be negative")
        self._id = id
        
    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, l):
        if l <= 0:
            raise ValueError("length cannot be zero or negative")
        self._length = l
    
    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, w):
        if w <= 0:
            raise ValueError("width cannot be zero or negative")
        self._width = w

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, h):
        if h <= 0:
            raise ValueError("height cannot be zero or negative")
        self._height = h
    
    def compute_fragility():
        pass

class Bin:
    def __init__(self, length, width, height, id, pose: PoseStamped = None):
        self.length = length
        self.width = width
        self.height = height
        self.height_map = np.zeros((length, width), dtype=int)
        self.priority_list = []
        self.boxes = {}
        
        # pose from Kevin
        self.id = id
        self.pose = pose
    
    """Kevin: these are for updating the poses for the box"""
    @property
    def pose(self):
        return self._pose

    @pose.setter
    def pose(self, pose: PoseStamped):
        self._pose = pose
        
    """Kevin: these are for updating the ids for the box"""
    @property
    def id(self):
        return self._id

    @id.setter
    def id(self, id):
        if id < 0:
            raise ValueError("I don't think id's can be negative")
        self._id = id
    
    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, l):
        if l <= 0:
            raise ValueError("length cannot be zero or negative")
        self._length = l
    
    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, w):
        if w <= 0:
            raise ValueError("width cannot be zero or negative")
        self._width = w

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, h):
        if h <= 0:
            raise ValueError("height cannot be zero or negative")
        self._height = h