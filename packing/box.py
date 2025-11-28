import numpy as np

class Box:
    def __init__(self, name, length, width, height, id, fragility=1.0):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.volume = self.length * self.width * self.height
        self.fragility = fragility
        
        # ID from Kevin:
        # "It seems like dynamic data like poses should be 
        # published and retrieved from topics, while static 
        # data like size could be defined in a shared package."
        self.id = id
    
    def __hash__(self):
      return hash((self.id, self.name))

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
    def __init__(self, name, length, width, height, id):
        self.name = name
        self.length = length
        self.width = width
        self.height = height
        self.height_map = np.zeros((length, width), dtype=int)
        self.priority_list = []
        self.boxes = {}
        
        # pose from Kevin
        self.id = id
    
    def __hash__(self):
      return hash((self.id, self.name))
    
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