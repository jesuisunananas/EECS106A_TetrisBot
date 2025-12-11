import numpy as np # pyright: ignore[reportMissingImports]
import uuid

class BigBox:
    def __init__(self, length, width, height, id, resolution = 0.01):
        self._length_m = float(length)
        self._width_m = float(width)
        self._height_m = float(height)
        self.length = length
        self.width = width
        self.height = height
        self.id = id
        self.resolution = resolution

    @property
    def length_m(self) -> float:
        return self._length_m

    @property
    def width_m(self) -> float:
        return self._width_m

    @property
    def height_m(self) -> float:
        return self._height_m
    
    def to_grid_units(self, value: float) -> int:
        return int(round(value / self.resolution))

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
    def volume(self):
        return self.length * self.width * self.height

class Box(BigBox):
    def __init__(self, length, width, height, id=-1, fragility=1.0, name=None):
        super().__init__(length, width, height, id)
        if name is None:
            if id != -1:
                name = str(id)
            else:
                name = f"box_{uuid.uuid4().hex[:8]}"  # short random ID
        self.name = name
        # fragility is a distribution between 0 to 1, 1 being not fragile

        self.fragility = fragility
        self.length = self.to_grid_units(self.length_m)
        self.width = self.to_grid_units(self.width_m)
        self.height = self.to_grid_units(self.height_m)

    @property
    def fragility(self):
        return self._fragility
    
    @fragility.setter
    def fragility(self, f):
        if f > 1 or f < 0:
            raise ValueError("fragility score must be between 0 and 1 inclusive")
        self._fragility = f    

class Bin(BigBox):
    def __init__(self, length, width, height, id=-1, name=None, resolution = 0.01):
        super().__init__(length, width, height, id) 
        self.resolution = resolution
        # absolute dimensions
        self.length = self.to_grid_units(self.length_m)
        self.width = self.to_grid_units(self.width_m)
        self.height = self.to_grid_units(self.height_m)
        self.height_map = np.zeros((self.length, self.width), dtype=int)
        self.priority_list = []
        self.boxes = {}
        
        if name is None:
            if id != -1:
                name = str(id)
            else:
                name = f"box_{uuid.uuid4().hex[:8]}"  # short random ID
        self.name = name
    
    # @property
    # def grid_length(self) -> int:
    #     return int(round(self.length / self.resolution))
    
    # @property
    # def grid_width(self) -> int:
    #     return int(round(self.width / self.resolution))

    # @property
    # def grid_shape(self) -> tuple[int, int]:
    #     return (self.grid_length, self.grid_width)

class Bundle(BigBox):
    def __init__(self, length, width, height, id=[], name=None):
        super().__init__(length, width, height, id) 
        if not isinstance(id, list): raise TypeError("Bundle objects should have a list of id's")
        # self.height_map = np.zeros((length, width), dtype=int)
        self.priority_list = []
        self.boxes = {}
        self.placed = False
        
        if name is None:
            if not id:
                name = str(id)
            else:
                name = f"box_{uuid.uuid4().hex[:8]}"  # short random ID
        self.name = name

    @property
    def placed(self):
        return self._placed
    
    @placed.setter
    def placed(self, status):
        # print(f"Bundle {self.name} is placed!")
        self._placed = status