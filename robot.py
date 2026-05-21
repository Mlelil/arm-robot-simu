import numpy as np
from math import cos, sin 
from typing import Tuple

class Robot:
    def __init__(self, arm_length:np.ndarray, joint_ranges:np.ndarray):
        self.length = arm_length
        self.ranges = joint_ranges


    def forward_kinematics(self, q:np.ndarray[float]) -> np.ndarray:
        """
        Computes the forward kinematic problem for the end effector
        """

        """voir le rapport mathématique pour les formules
        pour les conventions :
        pour i,j de snombres entiers:
        lij = li + lj
        cij = cos(qi + qj) / sij = sin(qi + qj)
        """

        if self.motion_allowed(q) == False :
            raise Exception("Le mouvement est impossible: q ne respecte pas les limites imposées")
        

        q1, q2, q3, _ = q
        _, l2, l3, l4 = self.length

        l34 = l3 + l4
        q23 = q2 + q3
        R = l2 * cos(q2) + l34 * cos(q23)

        x = cos(q1) * R
        y = sin(q1) * R
        z = l2 * sin(q2) + l34 * sin(q23)

        return np.array([x, y, z])

    def complete_forward_kinematics(self, q:np.ndarray[float]) -> np.ndarray:
        """
        Computes the forward kinematic problem for all the joints
        """

        if self.motion_allowed(q) == False :
            raise Exception("Le mouvement est impossible: q ne respecte pas les limites imposées")
        
        q1, q2, q3, q4 = q
        l1, l2, l3, l4 = self.length
        l34 = l3 + l4
        c1, s1 = cos(q1), sin(q1)
        c2, s2 = cos(q2), sin(q2)
        c23, s23 = cos(q2 + q3), sin(q2 + q3)

        xs, ys, zs = [0]*5, [0]*5, [0]*5 
        
        # p2 (Coude)
        xs[2] = l2 * c1 * c2
        ys[2] = l2 * s1 * c2
        zs[2] = l2      * s2
        # p3 (Poignet)
        xs[3] = xs[2] + l3 * c1 * c23
        ys[3] = ys[2] + l3 * s1 * c23
        zs[3] = zs[2] + l3      * s23
        # p4 (Effecteur)
        xs[4] = xs[2] + l34 * c1 * c23
        ys[4] = ys[2] + l34 * s1 * c23
        zs[4] = zs[2] + l34      * s23

        return xs, ys, zs

    def compute_rotation(self, q:np.ndarray) -> np.ndarray:
        """
        Computes the global rotational matrix of the arm
        """
        
        if self.motion_allowed(q) == False :
            raise Exception("Le mouvement est impossible: q ne respecte pas les limites imposées")

        # La matrice de rotation globale est simplifiée pour l'affichage
        q1, q2, q3, q4 = q
        q23 = q2 + q3
        c1, s1, c23, s23 = cos(q1), sin(q1), cos(q23), sin(q23)
        
        # Rotation résultante approximative pour l'affichage des axes
        R1 = np.array([[c1, -s1, 0], [s1, c1, 0], [0, 0, 1]])
        R23 = np.array([[c23, 0, -s23], [0, 1, 0], [s23, 0, c23]])
        R4 = np.array([[1, 0, 0], [0, cos(q4), -sin(q4)], [0, sin(q4), cos(q4)]])
        return R1 @ R23 @ R4

    def jacobian(self, q:np.ndarray) -> np.ndarray:
        """
        Computes the Jacobian matrix
        """

        if self.motion_allowed(q) == False :
            raise Exception("Le mouvement est impossible: q ne respecte pas les limites imposées")
        
        q1, q2, q3, _ = q
        _, l2, l3, l4 = self.length

        l34 = l3 + l4
        q23 = q2 + q3  

        c1, s1 = cos(q1), sin(q1)
        c2, s2 = cos(q2), sin(q2)
        c23, s23 = cos(q23), sin(q23)

        # R, R', R'' pour simplifier le calcul matriciel final
        R = l2 * c2 + l34 * c23
        R_p = l34 * c23
        R_pp = l2 * s2 + l34 * s23

        return np.array([
            [-s1 * R, -c1 * R_pp, -c1 * R_p,       0],
            [ c1 * R, -s1 * R_pp, -s1 * R_p,       0],
            [0,             R,        R_p,         0],
            [0,            s1,        s1,     c1*c23],
            [0,           -c1,        -c1,    s1*c23],
            [1,             0,         0,        s23]
        ])
    
    def motion_allowed(self, q:np.ndarray) -> bool:
        """
        Renvoie True si le mouvement est dans les bornes physiques 
        """
        
        def between(x:float, bornes:Tuple[float, float]):
            """
            Renvoie True si x est compris dans les bornes
            """
            a, b = min(bornes), max(bornes)
            return (a <= x and x <= b)
        

        logik = True    # contient la somme booléenne de tous les tests
        for i, qi in enumerate(q):
            logik = logik and between(qi, self.ranges[i])
        
        return logik



# ==========================================
# ZONE DE TEST
# ==========================================
if __name__ == "__main__":
    length = np.array([9, 15, 15, 10])
    ranges = [(-180, 180), (0, 180), (-180, 180), (-180, 180)]
    robot = Robot(length, ranges)

    # R: Ok
    q = np.array([0, 0, 0, 0])
    print("Mouvement autorisé" if robot.motion_allowed(q) else "Mouvement non autorisé")

    # R: non OK
    q = np.array([-1000, 0, 0, 0])
    print("Mouvement autorisé" if robot.motion_allowed(q) else "Mouvement non autorisé")

    # R: non ok
    q = np.array([-1000, -10, 0, 0])
    print("Mouvement autorisé" if robot.motion_allowed(q) else "Mouvement non autorisé")
