import pytest
from .box import Box, Bin

class TestBox:
    def test_zero_box(self):
        with pytest.raises(ValueError):
            Box(0,0,1,1,0,None)
    def test_neg_box(self):
        with pytest.raises(ValueError):
            Box(1,-1,1,1,0,None)

class TestBin:
    def test_zero_bin(self):
        with pytest.raises(ValueError):
            Bin(0,0,1,1,0,None)
    def test_neg_bin(self):
        with pytest.raises(ValueError):
            Bin(1,-1,1,1,0,None)