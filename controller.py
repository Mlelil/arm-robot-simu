import numpy as np
from numpy.typing import NDArray
from math import radians, degrees, cos, sin, sqrt
from time import time
from robot import Robot


default_tuning = {
    "K_p"                       : 4.0,
    "K_DLS"                     : 5.0,
    "K_secondary_singularity"   : 0.5,
    "K_secondary_joint"         : 5.0,
    "sigma_min_safe"            : 0.6,
    "max_norm"                  : 1.2
}


class Controller:
    def __init__(self, robot: Robot, tuning: dict[str, float]=default_tuning):
        self.robot = robot
        self.tuning = tuning                    # Variables de contrôle
        self.x0         : np.ndarray            # Position initiale du mouvement
        self.xf         : np.ndarray            # Position finale du mouvement
        self.t          : float                 # Instant t du mouvement
        self.T          : float                 # Durée totale du mouvement
        self.t_start    : float                 # Instant de départ
        self.t_prev     : float                 # Instant d'arrivée
        self.q_state    : NDArray[np.float64]   # Position articulaire à l'instant t
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

    def quintic_traj(self, t:float, T:float, X0:NDArray[np.float64], Xf:NDArray[np.float64]):
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


    def _sigma_min_gradient(self, q, eps=1e-5): #TODO clean
        """Calcule le gradient de w(q) = s_min(q)"""
        s0 = self.robot.sigma_min_value(q)    # TODO ajouter cette fonction dans la classe Robot
        grad = np.zeros_like(q)

        for i in range(len(grad)):
            dq = np.zeros_like(q)
            dq[i] = eps
            s1 = self.robot.sigma_min_value(q + dq)
            grad[i] = (s1 - s0) / eps
        
        return grad
    

    def _null_space_joint(self, ranges, k=5.0):
        """Optimisation dans l'espace nul de J_dls pour éviter les butées mécaniques"""
        # ATTENTION !!! on n'oublie pas qu'il faut prendre le GRADIENT de la fonction de coût
        q_min = np.array([range[0] for range in ranges])
        q_max = np.array([range[1] for range in ranges])

        q_min = np.radians(q_min)
        q_max = np.radians(q_max)

        temp = self.q_state - 0.5 * (q_max + q_min)
        range = 0.5 * (q_max - q_min)
        return - 2 * k * temp / (range **2)


    def _null_space_singularity(self, k):
        """Optimisation dans l'espace nul de J_dls pour éviter les singularités aka les positions avec sigma_min faible"""
        grad_sigma_min = self._sigma_min_gradient(self.q_state)
        activation = max(0, 1.0 - self.robot.sigma_min_value(self.q_state) / self.tuning["sigma_min_safe"]) # gain adaptatif : nul loin des singularités, croît quand on approche
        return k * activation

    def _dls_inverse(self, J, sigma_min):
        """Calcule  Pseudo inverse amortie adaptatif (adaptative DLS)"""
        if sigma_min >= self.tuning["sigma_min_safe"] :
            lam = 0
        else :
            lam = self.tuning["K_DLS"] * (1 - sigma_min / self.tuning["sigma_min_safe"])**2 #TODO essayer avec et sans le carré pour voir
        return  J.T @ np.linalg.inv(J @ J.T + lam * np.eye(3))



    def compute_qdot(self): #TODO couper cette fonction en plusieurs fonctions atomiques dans l'optique de les tuner différemment
        """ Calcule les vitesses articulaires à commander à instant t pour atteindre 
        les vitesses cartésiennes désirées
        Output:
            qdot : np.array(4, )

        """

        # On récupère l'état de la cinématique actuelle
        J = self.robot.jacobian(self.q_state)[0:3, :]     # Contrôle de la vitesse uniquement, pas de rotation
        X = self.robot.forward_kinematics(self.q_state)
        
        # Calcul du gain proportionnel effectif Kp (diminue près des singularités)
        sigma_min = self.robot.sigma_min_value(self.q_state)
        scale = min(1.0, sigma_min/self.tuning["sigma_min_safe"])
        Kp_eff = self.tuning["K_p"] * scale

        # On récupère la vitesse cartésienne désirée pour notre trajectoire
        X_desire, V_desire = self.quintic_traj(self.t, self.T, self.x0, self.xf)
        V = self.cartesian_controller(X, X_desire, V_desire, Kp_eff)

        # DLS inverse
        J_dls = self._dls_inverse(J, sigma_min)

        # Tâche secondaire pour éviter les butées mécaniques
        z = self._null_space_joint(self.robot.ranges, self.tuning["K_secondary_joint"])    #TODO à supprimer ou utiliser

        # Tâche secondaire pour éviter les singularités
        km_eff = self._null_space_singularity(self.tuning["K_secondary_singularity"])

        # Normalisation du gradiant pour la stabilité
        # norm_grad = np.linalg.norm(grad_sigma_min)
        # if norm_grad > 1e-8 :
        #     #print(f"Norm of grad_sigma_min = {norm_grad:.1f} and the qdot norm contribution is : {np.linalg.norm(km * NullSpace @ grad_smin)}")
        #     grad_sigma_min /= norm_grad


        # Calcul de qdot
        NullSpace = np.eye(4) - J_dls @ J
        grad_sigma_min = self._sigma_min_gradient(self.q_state)
        qdot = J_dls @ V + km_eff * NullSpace @ grad_sigma_min #+  NullSpace @ z

        # Limitation de qdot : on garde l'orientation du vecteur, mais on change sa norme
        norm = np.linalg.norm(qdot)
        max_norm = 1.2
        if norm > max_norm :
            qdot = qdot * (max_norm/norm)
        
        return 4




    def step(self, q:NDArray[np.float64], t:float):
        """ Calcule la vitesse articulaire q_dot à envoyer aux moteurs pour l'instant t+1
        """
        self.q_state = q
        self.t = t
        qdot = self.compute_qdot()
        self.q_dot = qdot 
        return qdot

    def is_motion_finished(self) -> bool:
        """ #TODO
        Renvoir True si le mouvement est terminé pour l'afficher ou débloquer les actions permises à l'arrêt
        """
        return True
