import numpy as np # pyright: ignore[reportMissingImports]
import uuid

class BigBox:
    def __init__(self, length, width, height, id, resolution = 0.01):
        self.length = length
        self.width = width
        self.height = height
        self.id = id
        self.resolution = resolution

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
        self.length = self.grid_length(self.resolution)
        self.width = self.grid_width(self.resolution)
        self.height = self.grid_height(self.resolution)

    @property
    def fragility(self):
        return self._fragility

    @property
    def grid_length(self, resolution) -> int:
        return int(round(self.length / resolution))
    
    @property
    def grid_width(self, resolution) -> int:
        return int(round(self.width / resolution))

    @property
    def grid_height(self, resolution) -> int:
        return int(round(self.height / resolution))
    
    @fragility.setter
    def fragility(self, f):
        if f > 1 or f < 0:
            raise ValueError("fragility score must be between 0 and 1 inclusive")
        self._fragility = f    

class Bin(BigBox):
    def __init__(self, length, width, height, id=-1, name=None, resolution = 0.01):
        super().__init__(length, width, height, id) 
        self.resolution = resolution
        H = self.grid_length
        W = self.grid_length


        self.height_map = np.zeros((H, W), dtype=int)
        self.priority_list = []
        self.boxes = {}
        
        if name is None:
            if id != -1:
                name = str(id)
            else:
                name = f"box_{uuid.uuid4().hex[:8]}"  # short random ID
        self.name = name
    
    @property
    def grid_length(self) -> int:
        return int(round(self.length / self.resolution))
    
    @property
    def grid_width(self) -> int:
        return int(round(self.width / self.resolution))

    @property
    def grid_shape(self) -> tuple[int, int]:
        return (self.grid_length, self.grid_width)

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