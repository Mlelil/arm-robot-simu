import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from robot import Robot
from controller import Controller
from history import MotionHistory
import sys
from time import time
from math import radians

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
    def _draw_robot(self, q): #TODO aussi en cours ici
        """Dessine le robot"""
        # 1. calculs cinématiques
        x_list, y_list, z_list = self.robot.complete_forward_kinematics(q)
        x_hand, y_hand, z_hand = x_list[-1], y_list[-1], z_list[-1]

        # 2. Gestion du tracé de la trajectoire

        # 3. Affichage matlplotlib
        self.ax.clear()

        # 4. Dessin du repère effecteur
        colors = ['#555555', '#E74C3C', '#2ECC71', '#3498DB']
        widths = [6, 5, 4, 3]
        for i in range(4):
            self.ax.plot(x_list[i:i+2], y_list[i:i+2], z_list[i:i+2], color=colors[i], linewidth=widths[i], solid_capstyle='round')
        self.ax.scatter(x_list, y_list, z_list, s=80, c='black', zorder=10)
        
        R_hand = self.robot.compute_rotation(q)
        length = 8
        self.ax.quiver(x_hand, y_hand, z_hand, R_hand[0,0], R_hand[1,0], R_hand[2,0], color='blue', length=length)
        self.ax.quiver(x_hand, y_hand, z_hand, R_hand[0,1], R_hand[1,1], R_hand[2,1], color='green', length=length)
        self.ax.quiver(x_hand, y_hand, z_hand, R_hand[0,2], R_hand[1,2], R_hand[2,2], color='red', length=length)


        limit = 45
        self.ax.set_xlim((-limit, limit)); self.ax.set_ylim((-limit, limit)); self.ax.set_zlim((0, limit))
        self.ax.set_xlabel('X'); self.ax.set_ylabel('Y'); self.ax.set_zlabel('Z')
        self.ax.set_title("Simulation Cinématique")
        
        self.canvas.draw()
        
        # 5. Mise à jour des données textuelles
        # self._update_data_display(q, hand_x, hand_y, hand_z)
        pass

    # --- Callback Slider ---
    def _update_data_from_sliders(self, *args):
        """Callback appelé lorsqu'un slider est utilisé
        Dessine le robot avec les angles souhaités"""
        #TODO erreur ici :
        # loop of ufunc does not support argument 0 of type method has no callable radians method - ok
        # 
        q = np.array([radians(var.get) for var in self.joint_vars])
        # self.controller.q_state = q je ne pense pas que ce soit utile ici
        self._draw_robot(q)
        # TODO
        # draw_robot(nouvelles valeurs manuelles)
        # update_données textuelles

    # --- Boucle auto ---
    def _auto_step(self): #TODO logique ici à faire
        if self.controller.motion_done :
            print("Motion over")
            return 
        
        # c'est le contrôleur qui va vérifier que le mouvement doit s'arrêter, est terminé etc...
        self.controller.step(time())    
        # TODO ajouter l'historique
        self.root.after(20, self._auto_step)

    # --- Callbacks Boutons ---
    def _apply_auto_command(self):
        """ Enregistre et lance la commande de déplacement automatique
        à partir d'une commande cible dans l'espace cartésien"""

        # On récupère les données entrées dans les champs X, Y, Z et 
        # on vérifie que l'objectif est atteignable au vu de notre robot.
        # Pour simplifier, on va supposer que tout point inclus dans 
        # la demisphère supérieur (z>0) de rayon <= longueur du bras est admissible

        try:
            x_target = float(self.entry_x.get())
            y_target = float(self.entry_y.get())
            z_target = float(self.entry_z.get())

            #TODO ça marche pas la formule
            rayon_demisphere_admissible = float(sum(robot.length[1:])) # la 1ere longueur est un entraxe et n'appartient pas vraiment au bras
            if z_target < 0 or x_target**2 + y_target**2 + z_target**2 > rayon_demisphere_admissible :
                raise AssertionError
            
            X = self.controller.get_pos()
            messagebox.showinfo("Commande Auto", f"Now at : (x,y,z) = ({X[0]}, {X[1]}, {X[2]})\nTarget at: ({x_target}, {y_target}, {z_target})")
            #TODO lancer un programme dessinant en pointillé une trajectoire théorique que la main peut emprunter : une droite, une polynomiale, la vraie
            self.controller.start_motion(time(), X, np.array([x_target, y_target, z_target]))
        
        except AssertionError:
            messagebox.showerror("Erreur", "Veuillez entrer une cible atteignable")
        except ValueError:
            messagebox.showerror("Erreur", "Veuillez entrer des nombres valides")

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
        sys.exit()
    
    # --- Builders privés ---
    def _build_window(self):
        """ Génère la fenêtre principale (root),
        ainsi que les styles utilisés
        """
        # Fenêtre principale
        self.root = tk.Tk()
        self.root.title('Simulateur de Bras Robotisé v3')
        self.root.geometry("1100x750")
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)

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
        self.ax = fig.add_subplot(111, projection='3d')
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

        self.joint_vars=[]
        for i, (min_value, max_value) in enumerate(self.robot.ranges):
            frame_s = ttk.Frame(sidebar)
            frame_s.pack(fill=tk.X, pady=2)

            lbl = ttk.Label(frame_s, text=f"θ{i+1}", width=5)
            lbl.pack(side=tk.LEFT)

            var = tk.DoubleVar(value=(max_value+min_value)/2)
            print(var) # TODO var n'est ptet pas un nombre
            self.joint_vars.append(var)

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

        self.entry_x = ttk.Entry(inputs_frame, width=8); self.entry_x.grid(row=0, column=1, pady=2)
        self.entry_y = ttk.Entry(inputs_frame, width=8); self.entry_y.grid(row=1, column=1, pady=2)
        self.entry_z = ttk.Entry(inputs_frame, width=8); self.entry_z.grid(row=2, column=1, pady=2)
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

        # --- Section 3 : Trajectoires ---
        ttk.Label(sidebar, text="Trajectoire", style='Title.TLabel').pack(anchor='w', pady=(0, 5))

        traj_tools = ttk.Frame(sidebar)
        traj_tools.pack(fill=tk.X, pady=5)

        # 3 boutons : Pinceau (Start), Pause, Poubelle
        btn_traj_play = tk.Button(traj_tools, text="✍", font=("Arial", 14), width=4, command=self._toggle_recording)
        btn_traj_play.pack(side=tk.LEFT, padx=2)
        btn_traj_pause = tk.Button(traj_tools, text="⏸", font=("Arial", 14), width=4, command=self._pause_recording)
        btn_traj_pause.pack(side=tk.LEFT, padx=2)
        btn_traj_del = tk.Button(traj_tools, text="🗑", font=("Arial", 14), width=4, command=self._clear_trajectory)
        btn_traj_del.pack(side=tk.LEFT, padx=2)

        ttk.Separator(sidebar, orient="horizontal").pack(fill='x', pady=15)

        # --- Section 4 : Données & Loupe ---
        data_frame = ttk.Frame(sidebar)
        data_frame.pack(fill=tk.X)
        ttk.Label(data_frame, text="Données", style='Title.TLabel').pack(side=tk.LEFT)

        # Bouton Loupe
        btn_loupe = tk.Button(data_frame, text="🔍", font=("Arial", 12), command=self._open_analysis_window)
        btn_loupe.pack(side=tk.RIGHT)

        # Zone de texte / données supplémentaires
        data_label = ttk.Label(sidebar, text="En attente de données...", style='Data.TLabel', anchor="nw", justify="left")
        data_label.pack(fill=tk.X, pady=5, ipady=10)

        # --- Section 5 : Bas de page (Reset/Stop) ---
        spacer = tk.Frame(sidebar, height=20, bg='#f0f0f0')
        spacer.pack(fill=tk.BOTH, expand=True)

        reset_btn = tk.Button(sidebar, text="🔄 RESET ROBOT", bg="#13c543", fg="white", font=("Arial", 10, "bold"), command=self._reset_robot)
        reset_btn.pack(fill=tk.X, pady=5)
        stop_btn = tk.Button(sidebar, text="⛔ ARRÊT D'URGENCE", bg="#d72f1d", fg="white", font=("Arial", 10, "bold"), command=self._emergency_stop)
        stop_btn.pack(fill=tk.X, pady=5)

    def _on_closing(self):
        """Appelée quand on appuie sur la croix qui ferme l'app"""
        print("Fermeture de l'application")
        self.root.quit()    # stop le mainloop
        self.root.destroy() # ferme les widgets tranquillement

    # --- Run ---
    def run(self):
        self.root.mainloop()
        sys.exit(0)

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