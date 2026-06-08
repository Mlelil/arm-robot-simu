import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from robot import Robot
from controller import Controller
from history import MotionHistory

class App:
    def __init__(self, robot: Robot, controller: Controller, history: MotionHistory):
        self.robot = robot
        self.controller = controller
        self.history = history

        # États GUI
        self.is_recording = False # par exemple

        # Construction dans l'ordre
        self._build_window()
        self._build_canvas()
        self._build_sidebar()
        self._draw_robot(controller.q_state)
        
    
    # --- Dessin ---
    def _draw_robot(self, q):
        pass

    # --- Callbacks Slider ---
    def _update_data_from_sliders(self, *args):
        pass

    # --- Boucle auto ---
    def _auto_step(self):
        pass

    # --- Callbacks Boutons ---
    def _apply_auto_command(self):
        pass
    def _draw_trajectory(self):
        pass
    def _clear_trajectory(self):
        pass
    def _toggle_recording(self):
        pass
    def _pause_recording(self):
        pass
    def _open_analysis_window(self):
        pass
    def _reset_robot(self):
        pass
    def _emergency_stop(self):
        pass
    
    # --- Builders privés ---
    def _build_window(self):
        """ Génère la fenêtre principale (root),
        ainsi que les styles utilisés
        """
        # Fenêtre principale
        self.root = tk.Tk()
        self.root.title('Simulateur de Bras Robotisé v3')
        self.root.geometry("1100x750")

        # Styles
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Title.TLabel', font=('Helvetica', 12, 'bold', 'underline'))
        style.configure('Data.TLabel', font=('Consolas', 9), background='#dddddd', padding=5)
    def _build_canvas(self):
        """ Génère toutes le fenêtres : #TODO meilleure déf
        figures, axe, canvas matplotlib
        """

    def _build_sidebar(self):
        """Génère la sidebar, et widgets tkinter
        """
        sidebar = tk.Frame(self.root, width=300, bg='#f0f0f0', padx=10, pady=10)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)
        sidebar.pack_propagate(False) # Force la largeur

    # --- Run ---
    def run(self):
        self.root.mainloop()


# --- Tests ---
if __name__ == "__main__":
    length = np.array([9, 15, 15, 10])
    ranges = np.array([(-180, 180), (0, 180), (-180, 180), (-180, 180)])
    robot = Robot(length, ranges)
    controller = Controller(robot)
    history = MotionHistory()

    print("Creation de l'application")
    app = App(robot, controller, history)
    app.run()