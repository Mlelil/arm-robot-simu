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
        """ Génère la fenêtre principale avec la visualisation 3D du bras :
        figures, axe, canvas matplotlib
        """
        fig = plt.figure(figsize=(7,7))
        ax = fig.add_subplot(111, projection='3d')
        self.canvas = FigureCanvasTkAgg(fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)


    def _build_sidebar(self):
        """Génère la sidebar, et widgets tkinter
        """
        sidebar = tk.Frame(self.root, width=300, bg='#f0f0f0', padx=10, pady=10)
        sidebar.pack(side=tk.LEFT, fill=tk.Y)
        sidebar.pack_propagate(False) # Force la largeur
        
        # --- Section 1 : Commande Manuelle ---
        ttk.Label(sidebar, text="Commande manuelle", style='Title.TLabel').pack(anchor='w', pady=(0, 10))

        joint_vars=[]
        for i, (min_value, max_value) in enumerate(self.robot.ranges):
            frame_s = ttk.Frame(sidebar)
            frame_s.pack(fill=tk.X, pady=2)

            lbl = ttk.Label(frame_s, text=f"θ{i+1}", width=4)
            lbl.pack(side=tk.LEFT)

            var = tk.DoubleVar(value=(max_value+min_value)/2)
            joint_vars.append(var)

            # Slider
            scl = ttk.Scale(frame_s, from_=min_value, to=max_value, orient=tk.HORIZONTAL, variable=var, command=self._update_data_from_sliders)
            scl.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            # Affichage valeur numérique
            val_lbl = ttk.Label(frame_s, text=f"{var.get()}°", width=6)
            val_lbl.pack(side=tk.LEFT)
            var.trace_add('write', lambda *args, l=val_lbl, x=var: l.config(text=f"{x.get():.1f}°"))
        
        ttk.Separator(sidebar, orient='horizontal').pack(fill='x', pady=15)

        # --- Section 2 : Commande Auto ---
        ttk.Label(sidebar, text="Commande auto", style='Title.TLabel').pack(anchor='w', pady=(0, 5))

        auto_frame = ttk.Frame(sidebar)
        auto_frame.pack(fill=tk.X)

        # Champs X, Y, Z
        inputs_frame = ttk.Frame(auto_frame)
        inputs_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)

        entry_x = ttk.Entry(inputs_frame, width=8); entry_x.grid(row=0, column=1, pady=2)
        entry_y = ttk.Entry(inputs_frame, width=8); entry_y.grid(row=1, column=1, pady=2)
        entry_z = ttk.Entry(inputs_frame, width=8); entry_z.grid(row=2, column=1, pady=2)
        ttk.Label(inputs_frame, text="x = ").grid(row=0, column=0)
        ttk.Label(inputs_frame, text="y = ").grid(row=1, column=0)
        ttk.Label(inputs_frame, text="z = ").grid(row=2, column=0)

        # Boutons Valider / Annuler (boutons carrés avec icônes)
        btn_frame = ttk.Frame(auto_frame)
        btn_frame.pack(side=tk.RIGHT, padx=5)

        btn_ok = tk.Button(btn_frame, text="✔", bg="#2ecc71", fg="white", font=("Arial", 12, "bold"), width=3, command=self._apply_auto_command)
        btn_ok.pack(pady=2)
        btn_cancel = tk.Button(btn_frame, text="✖", bg="#e74c3c", fg="white", font=("Arial", 12, "bold"), width=3, command=self._emergency_stop)
        btn_cancel.pack(pady=2)

        ttk.Separator(sidebar, orient="horizontal").pack(fill='x', pady=15)
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