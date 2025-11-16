import numpy as np

class Box:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height

    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, l):
        if l < 0:
            raise ValueError("length cannot be negative")
        self._length = l
    
    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, w):
        if w < 0:
            raise ValueError("width cannot be negative")
        self._width = w

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, h):
        if h < 0:
            raise ValueError("height cannot be negative")
        self._length = h

class Bin:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height
        self.height_map = np.zeros((length, width), dtype=int)
    
    @property
    def length(self):
        return self._length

    @length.setter
    def length(self, l):
        if l < 0:
            raise ValueError("length cannot be negative")
        self._length = l
    
    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, w):
        if w < 0:
            raise ValueError("width cannot be negative")
        self._width = w

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, h):
        if h < 0:
            raise ValueError("height cannot be negative")
        self._length = h