import sys
import os
import time
import tkinter as tk
import RPi.GPIO as GPIO
import board
import busio
import adafruit_vl53l0x  # Libreria per il nuovo sensore
from DFRobot_TMF8x01 import DFRobot_TMF8801 as tof

class DistanceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor Statimetri: Laser TMF + Laser VL53 + Ultrasuoni")
        self.root.geometry("1000x900") # Aumentata l'altezza per far spazio al terzo sensore
        self.root.configure(bg="#1e1e1e")

        # --- COSTANTI ---
        self.ALTEZZA_SENSORE_CM = 219.0
        self.NUM_LETTURE_ULTRASUONI = 5

        # --- PARAMETRI DI COMPENSAZIONE LINEARE (y = mx + q) ---
        self.M_LASER = 0.954826
        self.Q_LASER = 6.560159

        self.M_ULTRA = 1.019716
        self.Q_ULTRA = -3.550358

        # --- CONFIGURAZIONE GPIO E I2C ---
        self.PIN_TRIGGER = 17
        self.PIN_ECHO = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(self.PIN_ECHO, GPIO.IN)

        # Inizializzazione Bus I2C per VL53L0X
        self.i2c_bus = busio.I2C(board.SCL, board.SDA)

        # --- INIZIALIZZAZIONE SENSORI ---
        # 1. Laser TMF8801 (quello originale)
        self.sensor_tmf = tof(enPin = -1, intPin = -1, bus_id = 1)
        
        # 2. Laser VL53L0X (il nuovo)
        try:
            self.sensor_vl53 = adafruit_vl53l0x.VL53L0X(self.i2c_bus)
            self.vl53_attivo = True
        except Exception as e:
            print(f"Errore inizializzazione VL53L0X: {e}")
            self.vl53_attivo = False

        # --- INTERFACCIA GRAFICA ---
        tk.Label(root, text="SISTEMA DI MISURA ALTEZZA", font=("Arial", 26, "bold"),
                 bg="#1e1e1e", fg="white").pack(pady=10)

        # Frame Laser TMF8801 (Verde)
        self.frame_laser = tk.Frame(root, bg="#1e1e1e", highlightbackground="#28a745", highlightthickness=2)
        self.frame_laser.pack(pady=5, padx=20, fill="both")
        tk.Label(self.frame_laser, text="LASER TMF8801 (Originale)", font=("Arial", 14),
                 bg="#1e1e1e", fg="#28a745").pack()
        self.laser_var = tk.StringVar(value="---")
        self.label_laser = tk.Label(self.frame_laser, textvariable=self.laser_var,
                                    font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#28a745")
        self.label_laser.pack()

        # Frame Laser VL53L0X (Arancione) - IL NUOVO SENSORE
        self.frame_vl53 = tk.Frame(root, bg="#1e1e1e", highlightbackground="#ff8c00", highlightthickness=2)
        self.frame_vl53.pack(pady=5, padx=20, fill="both")
        tk.Label(self.frame_vl53, text="LASER VL53L0X (Nuovo)", font=("Arial", 14),
                 bg="#1e1e1e", fg="#ff8c00").pack()
        self.vl53_var = tk.StringVar(value="---")
        self.label_vl53 = tk.Label(self.frame_vl53, textvariable=self.vl53_var,
                                   font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#ff8c00")
        self.label_vl53.pack()

        # Frame Ultrasuoni (Blu)
        self.frame_ultra = tk.Frame(root, bg="#1e1e1e", highlightbackground="#007bff", highlightthickness=2)
        self.frame_ultra.pack(pady=5, padx=20, fill="both")
        tk.Label(self.frame_ultra, text="ULTRASUONI HC-SR04", font=("Arial", 14),
                 bg="#1e1e1e", fg="#007bff").pack()
        self.ultra_var = tk.StringVar(value="---")
        self.label_ultra = tk.Label(self.frame_ultra, textvariable=self.ultra_var,
                                    font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#007bff")
        self.label_ultra.pack()

        # Avvio sensore TMF
        if self.sensor_tmf.begin() == 0:
            self.sensor_tmf.start_measurement(calib_m = self.sensor_tmf.eMODE_CALIB)
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
                if start - t0 > 0.05: break
            t0 = time.time()
            while GPIO.input(self.PIN_ECHO) == 1:
                stop = time.time()
                if stop - t0 > 0.05: break
            dist = ((stop - start) * 34300) / 2
            if 2.0 < dist < self.ALTEZZA_SENSORE_CM:
                letture.append(dist)
            time.sleep(0.01)
        return min(letture) if letture else None

    def update_all(self):
        # 1. Lettura LASER TMF8801
        if self.sensor_tmf.is_data_ready():
            dist_cm = self.sensor_tmf.get_distance_mm() / 10.0
            altezza_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_cm)
            altezza_comp = (altezza_raw * self.M_LASER) + self.Q_LASER
            self.laser_var.set(f"{altezza_comp:.1f} cm")

        # 2. Lettura LASER VL53L0X (Nuovo)
        if self.vl53_attivo:
            try:
                # Lettura in mm trasformata in cm
                dist_vl_cm = self.sensor_vl53.range / 10.0
                altezza_vl_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_vl_cm)
                # Per ora usiamo calibrazione neutra (1:1), puoi correggerla dopo
                self.vl53_var.set(f"{altezza_vl_raw:.1f} cm")
            except Exception:
                self.vl53_var.set("ERR")

        # 3. Lettura ULTRASUONI
        dist_u = self.get_ultrasuoni_filtrato()
        if dist_u:
            altezza_u_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_u - 3)
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
