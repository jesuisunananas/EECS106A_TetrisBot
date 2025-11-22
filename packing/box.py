import numpy as np
from scipy.spatial.transform import RigidTransform, Rotation

class Box:
    def __init__(self, length, width, height):
        self.length = length
        self.width = width
        self.height = height
        
        # Box properties:
        self.volume = length * width * height
        self.density = 1.0 # probably porportional to fragility?
        self.access_priority = 0.0
        self.id = 0
        self.tf # maybe from scipy.spatial.transform.RigidTransform
        self.rotation # maybe from scipy.spatial.transform.Rotation

    @property
    def mass(self):
        return self._density * self._length * self._width * self._height
    
    @property
    def volume(self):
        return self._length * self._width * self._height

        
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


    @property
    def density(self):
        return self._density
    
    @density.setter
    def volume(self, density):
        if density <= 0:
            raise ValueError("density cannot be zero or negative")
        self._density = density



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