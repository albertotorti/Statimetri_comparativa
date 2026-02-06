import sys
import os
import time
import tkinter as tk
import RPi.GPIO as GPIO 
from DFRobot_TMF8x01 import DFRobot_TMF8801 as tof

class DistanceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor Statimetri: Laser + Ultrasuoni")
        self.root.geometry("1000x700")
        self.root.configure(bg="#1e1e1e")

        # --- COSTANTI ---
        self.ALTEZZA_SENSORE_CM = 219.0 
        self.NUM_LETTURE_ULTRASUONI = 5  
        
        # --- PARAMETRI DI COMPENSAZIONE LINEARE (y = mx + q) ---
        # Calcolati su tutta la banda di misura per la massima fluidità
        self.M_LASER = 0.954826
        self.Q_LASER = 6.560159
        
        self.M_ULTRA = 1.019716
        self.Q_ULTRA = -3.550358

        # --- CONFIGURAZIONE GPIO ---
        self.PIN_TRIGGER = 17
        self.PIN_ECHO = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(self.PIN_ECHO, GPIO.IN)

        # --- INIZIALIZZAZIONE LASER ---
        self.sensor_laser = tof(enPin = -1, intPin = -1, bus_id = 1)
        
        # --- INTERFACCIA GRAFICA ---
        tk.Label(root, text="SISTEMA DI MISURA ALTEZZA", font=("Arial", 30, "bold"), 
                 bg="#1e1e1e", fg="white").pack(pady=20)

        # Frame Laser
        self.frame_laser = tk.Frame(root, bg="#1e1e1e", highlightbackground="#28a745", highlightthickness=2)
        self.frame_laser.pack(pady=10, padx=20, fill="both")
        tk.Label(self.frame_laser, text="STATIMETRO LASER - Lineare Unica", font=("Arial", 18), 
                 bg="#1e1e1e", fg="#28a745").pack(pady=5)
        self.laser_var = tk.StringVar(value="---")
        self.label_laser = tk.Label(self.frame_laser, textvariable=self.laser_var, 
                                    font=("Arial", 120, "bold"), bg="#1e1e1e", fg="#28a745")
        self.label_laser.pack()

        # Frame Ultrasuoni
        self.frame_ultra = tk.Frame(root, bg="#1e1e1e", highlightbackground="#007bff", highlightthickness=2)
        self.frame_ultra.pack(pady=10, padx=20, fill="both")
        tk.Label(self.frame_ultra, text="STATIMETRO ULTRASUONI - Lineare Unica", font=("Arial", 18), 
                 bg="#1e1e1e", fg="#007bff").pack(pady=5)
        self.ultra_var = tk.StringVar(value="---")
        self.label_ultra = tk.Label(self.frame_ultra, textvariable=self.ultra_var, 
                                    font=("Arial", 120, "bold"), bg="#1e1e1e", fg="#007bff")
        self.label_ultra.pack()

        if self.sensor_laser.begin() == 0:
            self.sensor_laser.start_measurement(calib_m = self.sensor_laser.eMODE_CALIB)
        else:
            self.laser_var.set("ERR")
        
        self.update_all()

    def get_ultrasuoni_filtrato(self):
        letture = []
        for _ in range(self.NUM_LETTURE_ULTRASUONI):
            GPIO.output(self.PIN_TRIGGER, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.PIN_TRIGGER, GPIO.LOW)
            
            start = time.time()
            stop = time.time()
            t0 = time.time()
            while GPIO.input(self.PIN_ECHO) == 0:
                start = time.time()
                if start - t0 > 0.1: break
            t0 = time.time()
            while GPIO.input(self.PIN_ECHO) == 1:
                stop = time.time()
                if stop - t0 > 0.1: break
            
            dist = ((stop - start) * 34300) / 2
            if 2.0 < dist < self.ALTEZZA_SENSORE_CM:
                letture.append(dist)
            time.sleep(0.02)
        return min(letture) if letture else None

    def update_all(self):
        # 1. Calibrazione LASER (y = mx + q)
        if self.sensor_laser.is_data_ready():
            dist_cm = self.sensor_laser.get_distance_mm() / 10.0
            altezza_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_cm)
            # Applicazione della retta di regressione
            altezza_comp = (altezza_raw * self.M_LASER) + self.Q_LASER
            self.laser_var.set(f"{altezza_comp:.1f} cm")

        # 2. Calibrazione ULTRASUONI (y = mx + q)
        dist_u = self.get_ultrasuoni_filtrato()
        if dist_u:
            # Calcolo base con offset hardware
            altezza_u_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_u - 3)
            # Applicazione della retta di regressione
            altezza_u_comp = (altezza_u_raw * self.M_ULTRA) + self.Q_ULTRA
            self.ultra_var.set(f"{altezza_u_comp:.1f} cm")
        else:
            self.ultra_var.set("---")

        self.root.after(200, self.update_all)

def chiudi():
    GPIO.cleanup()
    root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DistanceApp(root)
    root.protocol("WM_DELETE_WINDOW", chiudi)
    root.mainloop()
