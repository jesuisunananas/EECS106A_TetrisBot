import numpy as np # type: ignore
import pytest # type: ignore
from .box import Box, Bin
from .heuristics import place_box_with_rule

"""I think these ones are outdated, now that I've added some more parameters..."""

class TestPlacementSequence:

    def test_multiple_box_placements(self):
        # Create a new bin
        bin = Bin(4, 4, 5)

        # Boxes to place
        b  = Box(1, 1, 5)
        b1 = Box(1, 2, 2)
        b2 = Box(2, 3, 1)
        b3 = Box(2, 2, 2)

        # Initial height map
        print("\nInitial height_map:\n", bin.height_map)

        # Place boxes one by one
        placement_b  = place_box_with_rule(b,  bin)
        placement_b1 = place_box_with_rule(b1, bin)
        placement_b2 = place_box_with_rule(b2, bin)
        placement_b3 = place_box_with_rule(b3, bin)

        print("Placement b :",  placement_b)
        print("Placement b1:", placement_b1)
        print("Placement b2:", placement_b2)
        print("Placement b3:", placement_b3)

        print("Final height_map:\n", bin.height_map)

        # ---- Assertions ----
        # These depend on the exact rules of your packing logic.
        # You can fill in expected values below.

        assert placement_b  is not None
        assert placement_b1 is not None
        assert placement_b2 is not None
        assert placement_b3 is not None

        # Example structural checks:
        assert bin.height_map.shape == (4, 4)
        assert bin.height_map.max() <= bin.height

        # Optional: add exact height_map comparison once you know expected result
        # expected_map = np.array([...])
        # np.testing.assert_array_equal(bin.height_map, expected_map)
