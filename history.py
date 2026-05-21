from collections import deque   # Utilisation de deque car 1. matplotlib connaît 2. facile de gérer la taille de chaque liste car gérée automatiquement
import numpy as np
from typing import Dict


class MotionHistory:
    def __init__(self, maxlen: int = 500):
        self.maxlen = maxlen
        self.history = {
            "t":deque(maxlen=maxlen),

            "x":deque(maxlen=maxlen),
            "y":deque(maxlen=maxlen),
            "z":deque(maxlen=maxlen),

            "vx":deque([0], maxlen=maxlen),     # Initialisation à [0] car vx(t=0)=0
            "vy":deque([0], maxlen=maxlen),
            "vz":deque([0], maxlen=maxlen),

            "q1":deque(maxlen=maxlen),
            "q2":deque(maxlen=maxlen),
            "q3":deque(maxlen=maxlen),
            "q4":deque(maxlen=maxlen),
            
            "q1_dot":deque([0], maxlen=maxlen),  
            "q2_dot":deque([0], maxlen=maxlen),
            "q3_dot":deque([0], maxlen=maxlen),
            "q4_dot":deque([0], maxlen=maxlen),
                  
            "sigma_min":deque(maxlen=maxlen),
            "sigma_max":deque(maxlen=maxlen),
            "cond_J":deque(maxlen=maxlen),
            "det(JJT)":deque(maxlen=maxlen),
            "error_norm":deque(maxlen=maxlen),
            "||q_dot||":deque([0], maxlen=maxlen)
        } 



    def stocker(self, t:int, pos:np.ndarray, vel:np.ndarray, q:np.ndarray, qdot:np.ndarray, metrics:Dict[str, float]):
        """
        Stocke une nouvelle ligne de données dans l'historique.
        
        Paramètres:
            t (int): Pas de temps actuel
            pos (np.ndarray): Tableau [x, y, z] de la position
            vel (np.ndarray): Tableau [vx, vy, vz] de la vitesse cartésienne
            q (np.ndarray): Tableau [q1, q2, q3, q4] des angles
            qdot (np.ndarray): Tableau [q1_dot, q2_dot, q3_dot, q4_dot] des vitesses articulaires
            metrics (Dict): Dictionnaire contenant les calculs (sigma, conditionnement, etc...)
        """

        self.history["t"].append(t)
        self.history["x"].append(pos[0])
        self.history["y"].append(pos[1])
        self.history["z"].append(pos[2])
        self.history["vx"].append(vel[0])
        self.history["vy"].append(vel[1])
        self.history["vz"].append(vel[2])
        self.history["q1"].append(q[0])
        self.history["q2"].append(q[1])
        self.history["q3"].append(q[2])
        self.history["q4"].append(q[3])
        self.history["q1_dot"].append(qdot[0])
        self.history["q2_dot"].append(qdot[1])
        self.history["q3_dot"].append(qdot[2])
        self.history["q4_dot"].append(qdot[3])
        self.history["sigma_min"].append(metrics["sigma_min"])
        self.history["sigma_max"].append(metrics["sigma_max"])
        self.history["cond_J"].append(metrics["cond_J"])
        self.history["det(JJT)"].append(metrics["det(JJT)"])
        self.history["error_norm"].append(metrics["error_norm"])
        self.history["||q_dot||"].append(metrics["||q_dot||"])


    def get(self) -> Dict[str, float]:
        """
        Renvoie l'historique des valeurs pour l'affichage des courbes
        """
        return self.history




# ==========================================
# ZONE DE TEST
# ==========================================
if __name__ == "__main__":
    motion = MotionHistory()
    
    # On simule les fausses données
    t_test = 0.5
    pos_test = np.array([10.0, 5.0, 2.0])
    vel_test = np.array([0.1, 0.0, -0.1])
    q_test = np.array([1.57, 0.5, -0.5, 0.0])
    qdot_test = np.array([0.01, 0.02, 0.0, 0.0])
    metrics_test = {
        "sigma_min": 0.5,
        "sigma_max": 2.0,
        "cond_J": 4.0,
        "det(JJT)": 1.5,
        "error_norm": 0.05,
        "||q_dot||": 0.022
    }
    
    # On teste sotcker
    motion.stocker(t_test, pos_test, vel_test, q_test, qdot_test, metrics_test)
    
    # On teste get
    data = motion.get()
    print("Temps enregistré :", data["t"])
    print("Position X enregistrée :", data["x"])
    print("Sigma Min enregistré :", data["sigma_min"])