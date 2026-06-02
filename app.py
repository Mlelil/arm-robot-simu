import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class App:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Simulateur de Bras Robotique v3")
        self.root.geometry("1100x750")

        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('Title.TLabel', font=('Helvetica', 12, 'bold', 'underline'))
        self.style.configure('Data.TLabel', font=('Consolad', 9), background='#dddddd', padding=5)

        # Conteneur de gauche (contrôles, largeur fixe)
        self.sidebar = 3

        # Conteneur de droite : Simulation 3D

        #etc...


