import sys
import os
import time
import tkinter as tk
import RPi.GPIO as GPIO
import board
import busio
import adafruit_vl53l0x
from DFRobot_TMF8x01 import DFRobot_TMF8801 as tof

class DistanceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Monitor Statimetri: Lettura Sequenziale Pura")
        self.root.geometry("1280x700")
        self.root.configure(bg="#1e1e1e")

        # --- COSTANTI ---
        self.ALTEZZA_SENSORE_CM = 219.0
        self.NUM_LETTURE_ULTRASUONI = 5

        # --- PARAMETRI DI COMPENSAZIONE ---
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

        self.i2c_bus = busio.I2C(board.SCL, board.SDA)

        # --- INIZIALIZZAZIONE SENSORI ---
        # 1. Laser TMF8801
        self.sensor_tmf = tof(enPin = -1, intPin = -1, bus_id = 1)
        
        # 2. Laser VL53L0X
        try:
            self.sensor_vl53 = adafruit_vl53l0x.VL53L0X(self.i2c_bus)
            # Riduciamo il tempo di campionamento per velocizzare la sequenza
            self.sensor_vl53.measurement_timing_budget = 33000 # 33ms
            self.vl53_attivo = True
        except Exception as e:
            print(f"Errore VL53L0X: {e}")
            self.vl53_attivo = False

        # --- INTERFACCIA GRAFICA ---
        tk.Label(root, text="COMPARATIVA SEQUENZIALE", font=("Arial", 26, "bold"),
                 bg="#1e1e1e", fg="white").pack(pady=10)

        # Frame TMF8801
        self.frame_laser = tk.Frame(root, bg="#1e1e1e", highlightbackground="#28a745", highlightthickness=2)
        self.frame_laser.pack(pady=5, padx=20, fill="both")
        self.laser_var = tk.StringVar(value="---")
        tk.Label(self.frame_laser, textvariable=self.laser_var, font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#28a745").pack()

        # Frame VL53L0X
        self.frame_vl53 = tk.Frame(root, bg="#1e1e1e", highlightbackground="#ff8c00", highlightthickness=2)
        self.frame_vl53.pack(pady=5, padx=20, fill="both")
        self.vl53_var = tk.StringVar(value="---")
        tk.Label(self.frame_vl53, textvariable=self.vl53_var, font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#ff8c00").pack()

        # Frame Ultrasuoni
        self.frame_ultra = tk.Frame(root, bg="#1e1e1e", highlightbackground="#007bff", highlightthickness=2)
        self.frame_ultra.pack(pady=5, padx=20, fill="both")
        self.ultra_var = tk.StringVar(value="---")
        tk.Label(self.frame_ultra, textvariable=self.ultra_var, font=("Arial", 80, "bold"), bg="#1e1e1e", fg="#007bff").pack()

        if self.sensor_tmf.begin() == 0:
            self.sensor_tmf.start_measurement(calib_m = self.sensor_tmf.eMODE_CALIB)
        else:
            self.laser_var.set("ERR TMF")

        self.update_all()

    def get_ultrasuoni_filtrato(self):
        # Misura ultrasuoni isolata
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
                if start - t0 > 0.04: break
            t0 = time.time()
            while GPIO.input(self.PIN_ECHO) == 1:
                stop = time.time()
                if stop - t0 > 0.04: break
            dist = ((stop - start) * 34300) / 2
            if 2.0 < dist < self.ALTEZZA_SENSORE_CM:
                letture.append(dist)
            time.sleep(0.01)
        return min(letture) if letture else None

    def update_all(self):
        """ Ciclo di lettura sequenziale: TMF -> VL53 -> HC-SR04 """
        
        # 1. Lettura TMF8801
        # Chiediamo il dato solo se pronto per evitare di bloccare il bus troppo a lungo
        if self.sensor_tmf.is_data_ready():
            dist_tmf_mm = self.sensor_tmf.get_distance_mm()
            dist_cm = dist_tmf_mm / 10.0
            altezza_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_cm)
            altezza_comp = (altezza_raw * self.M_LASER) + self.Q_LASER
            self.laser_var.set(f"TMF: {altezza_comp:.1f} cm")
        
        # Piccola pausa per lasciare che i fotoni del TMF si "esauriscano"
        time.sleep(0.05)

        # 2. Lettura VL53L0X
        if self.vl53_attivo:
            try:
                # La libreria di Adafruit gestisce internamente l'attesa del raggio
                dist_vl_mm = self.sensor_vl53.range
                dist_vl_cm = dist_vl_mm / 10.0
                altezza_vl_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_vl_cm)
                self.vl53_var.set(f"VL53: {altezza_vl_raw:.1f} cm")
            except Exception:
                self.vl53_var.set("ERR VL53")

        # Altra pausa prima di attivare gli ultrasuoni (rumore elettrico/ottico)
        time.sleep(0.05)

        # 3. Lettura Ultrasuoni
        dist_u = self.get_ultrasuoni_filtrato()
        if dist_u:
            altezza_u_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_u - 3)
            altezza_u_comp = (altezza_u_raw * self.M_ULTRA) + self.Q_ULTRA
            self.ultra_var.set(f"ULTRA: {altezza_u_comp:.1f} cm")
        else:
            self.ultra_var.set("ULTRA: ---")

        # Il prossimo ciclo avverrà tra 100ms, garantendo un refresh fluido ma ordinato
        self.root.after(100, self.update_all)

def chiudi():
    GPIO.cleanup()
    root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = DistanceApp(root)
    root.protocol("WM_DELETE_WINDOW", chiudi)
    root.mainloop()
