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
    def _build_window(self): # root, styles
        pass
    def _build_canvas(self): # fig, axes, canvas matplotlib
        pass
    def _build_sidebar(self): # widgets tkinter
        pass

    # --- Run ---
    def run(self):
        self.root.mainloop()

