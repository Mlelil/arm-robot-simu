import numpy as np
from numpy.typing import NDArray
from math import radians, degrees, cos, sin, sqrt
from time import time

class Controller:
    def __init__(self, tuning:float=4.0):
        self.tuning = tuning                    # A définir, variables des filtres
        self.x0         : np.ndarray            # Position initiale du mouvement
        self.xf         : np.ndarray            # Position finale du mouvement
        self.T          : float                 # Durée totale du mouvement
        self.t_start    : float                 # Instant de départ
        self.t_prev     : float                 # Instant d'arrivée
        self.q          : NDArray[np.float64]   # Position articulaire à l'instant t
        self.old_qdot   : NDArray[np.float64]   # Position articulaire à l'instant t-1

    def start_motion(self, x0:NDArray[np.float64], xf:NDArray[np.float64], vmax=2.0):
        """Initialise toutes les variables pour commencer le mouvement
        à partir de la position actuelle, de la position finale et de la vitesse
        maximale lors du mouvement"""

        self.x0 = x0
        self.xf = xf
        self.T = max(float(np.linalg.norm(xf-x0)) / vmax, 1.0)
        self.t_start = time()
        self.t_prev = self.t_start


    def quintic_traj(self, t:np.float64, T:np.float64, X0:NDArray[np.float64], Xf:NDArray[np.float64]):
        """ Calcule la consigne de position et de vitesse à un instant t pour aller
        de la position X0 à Xf en T seondes avec une trajectoir C²([0,T])

        Inputs
            t : float(0 <= t <= T)
            T : float
            X0: np.array(3,)
            Xf: np.array(3,)

        Outputs:
            X_desire : np.array(3,)
            x_dot_d : np.array(3,)
        """

        s = t / T
        h = 10*s**3 - 15*s**4 + 6*s**5
        h_dot = 30*s**2 - 60*s**3 + 30*s**4

        X_desire = X0 + h * (Xf - X0) 
        V_desire = h_dot * (Xf -X0) / T

        return X_desire, V_desire


    def cartesian_controller(self, X:np.ndarray, X_desire:np.ndarray, V_desire:np.ndarray, Kp:float):
        """ Calcule la vitesse à donner à la main, pour répondre à la consigne de position X_desire et de vitesse V_desire
        Inputs:
            X           : np.array(3,)
            X_desire    : np.array(3,)
            V_desire    : np.array(3,)
            Kp          : float ou np.array(3, 3) si augmentation de la complexité du modèle
        Output:
            V           : np.array(3,)
        """

        # On contrôle encore dans le monde cartésien.
        # L'objectif suivant est de passer de la vitesse de la main, aux vitesses articulaires
        # nécessaires pour y arriver.
        return V_desire + Kp * (X_desire - X)


    def compute_qdot(self):
        """ Calcule les vitesses articulaires pour atteindre V_desire   TODO
        Inputs:
            q
            t
            T
            x0
            xf


        Output:

        """

    def null_space_optimization(self):
        #TODO
        pass

    def DLS(self):
        #TODO
        pass

    def step(self, q:NDArray[np.float64], t:float):
        """ Calcule la vitesse articulaire q_dot à envoyer aux moteurs pour l'instant t+1
        """
        pass

    def is_motion_finished(self) -> bool:
        """ #TODO
        Renvoir True si le mouvement est terminé pour l'afficher ou débloquer les actions permises à l'arrêt
        """
        return True
