import numpy as np


class ContactSchedule:
    """
    Describes the contact schedule of the robot.
    Holds the following information:
    -  When a given end effector is or is not in contact with a surface
    - The z height of the surface
    - A polytope description of the surface

    TODO: in the future we will need to support surfaces that are not parallel to the ground
    """
    def __init__(self):
        self.swing_times = []      # TODO: What is the best pythonic data structure for holding pairs (start and end times)

        self.A_poly = []
        self.b_poly = []

        self.swing_heights = []

    def in_contact(self, time: float, ee_idx: int) -> bool:
        """Return the contact state of a given end effector at a given time."""
        # TODO: Implement
        return False

    def get_poly_b(self, time: float, ee_idx: int) -> np.array:
        """Return the b vector in Ax <= b description of the polytope"""
        # TODO: Implement
        return np.zeros(3)

    def pass_time(self, time_passed: float):
        """Update the swing times to match the passed time."""

