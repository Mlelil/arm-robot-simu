import numpy as np

class Robot:
    def __init__(self, arm_length:np.ndarray, joint_ranges:np.ndarray):
        self.length = arm_length
        self.l1, self.l2, self.l3, self.l4 = arm_length

    def forward_kinematics(self, q:np.ndarray) -> np.ndarray:
        """
        Computes the forward kinematic problem
        """
        pass

    def jacobian(self, q:np.ndarray) -> np.ndarray:
        """
        Computes the Jacobian matrix
        """
        pass
