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
        self.root.title("Monitor Statimetri: Comparativa ToF")
        self.root.geometry("1280x700")
        self.root.configure(bg="#1e1e1e")

        # --- COSTANTI E STORICO ---
        self.ALTEZZA_SENSORE_CM = 219.0
        self.NUM_LETTURE_ULTRASUONI = 5
        self.vl53_history = []

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
        self.sensor_tmf = tof(enPin = -1, intPin = -1, bus_id = 1)
        
        try:
            self.sensor_vl53 = adafruit_vl53l0x.VL53L0X(self.i2c_bus)
            self.sensor_vl53.measurement_timing_budget = 200000 
            self.vl53_attivo = True
        except Exception as e:
            print(f"Errore VL53L0X: {e}")
            self.vl53_attivo = False

        # --- INTERFACCIA GRAFICA ---
        tk.Label(root, text="COMPARATIVA STATIMETRI", font=("Arial", 26, "bold"),
                 bg="#1e1e1e", fg="white").pack(pady=10)

        # Frame TMF8801
        self.frame_laser = tk.Frame(root, bg="#1e1e1e", highlightbackground="#28a745", highlightthickness=2)
        self.frame_laser.pack(pady=5, padx=20, fill="both")
        self.laser_var = tk.StringVar(value="---")
        tk.Label(self.frame_laser, textvariable=self.laser_var, font=("Arial", 70, "bold"), bg="#1e1e1e", fg="#28a745").pack()

        # Frame VL53L0X
        self.frame_vl53 = tk.Frame(root, bg="#1e1e1e", highlightbackground="#ff8c00", highlightthickness=2)
        self.frame_vl53.pack(pady=5, padx=20, fill="both")
        self.vl53_var = tk.StringVar(value="---")
        tk.Label(self.frame_vl53, textvariable=self.vl53_var, font=("Arial", 70, "bold"), bg="#1e1e1e", fg="#ff8c00").pack()

        # Frame Ultrasuoni
        self.frame_ultra = tk.Frame(root, bg="#1e1e1e", highlightbackground="#007bff", highlightthickness=2)
        self.frame_ultra.pack(pady=5, padx=20, fill="both")
        self.ultra_var = tk.StringVar(value="---")
        tk.Label(self.frame_ultra, textvariable=self.ultra_var, font=("Arial", 70, "bold"), bg="#1e1e1e", fg="#007bff").pack()

        # --- TASTO USCITA ---
        self.btn_esci = tk.Button(root, text="ESCI DALL'APPLICAZIONE", font=("Arial", 24, "bold"),
                                  bg="#dc3545", fg="white", activebackground="#a71d2a",
                                  command=self.chiudi_app)
        self.btn_esci.pack(pady=20, padx=20, fill="x")

        if self.sensor_tmf.begin() == 0:
            self.sensor_tmf.start_measurement(calib_m = self.sensor_tmf.eMODE_CALIB)
        else:
            self.laser_var.set("ERR TMF")

        self.update_all()

    def get_ultrasuoni_filtrato(self):
        letture = []
        for _ in range(self.NUM_LETTURE_ULTRASUONI):
            GPIO.output(self.PIN_TRIGGER, GPIO.HIGH)
            time.sleep(0.00001)
            GPIO.output(self.PIN_TRIGGER, GPIO.LOW)
            start, stop = time.time(), time.time()
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
        # 1. TMF8801
        if self.sensor_tmf.is_data_ready():
            dist_cm = self.sensor_tmf.get_distance_mm() / 10.0
            altezza_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_cm)
            altezza_comp = (altezza_raw * self.M_LASER) + self.Q_LASER
            self.laser_var.set(f"TMF: {altezza_comp:.1f} cm")
        
        # 2. VL53L0X
        if self.vl53_attivo:
            try:
                # Leggiamo la distanza
                val = self.sensor_vl53.range
                
                # Il VL53L0X restituisce valori sopra 8000 se non vede nulla
                # Inoltre, se la distanza è troppo alta per il tuo statimetro (es. > 219cm), lo azzeriamo
                if val < 8000 and (val / 10.0) < self.ALTEZZA_SENSORE_CM:
                    self.vl53_history.append(val)
                    if len(self.vl53_history) > 5:
                        self.vl53_history.pop(0)
                    
                    media_cm = (sum(self.vl53_history) / len(self.vl53_history)) / 10.0
                    self.vl53_var.set(f"VL53: {max(0, self.ALTEZZA_SENSORE_CM - media_cm):.1f} cm")
                else:
                    # Se non vede nulla o il valore è fuori scala, puliamo lo storico e resettiamo
                    self.vl53_history.clear()
                    self.vl53_var.set("VL53: ---") # Oppure "VL53: 0.0 cm"
            except:
                self.vl53_var.set("ERR VL53")

        # 3. Ultrasuoni
        dist_u = self.get_ultrasuoni_filtrato()
        if dist_u:
            altezza_u_raw = max(0, self.ALTEZZA_SENSORE_CM - dist_u - 3)
            altezza_u_comp = (altezza_u_raw * self.M_ULTRA) + self.Q_ULTRA
            self.ultra_var.set(f"ULTRA: {altezza_u_comp:.1f} cm")
        else:
            self.ultra_var.set("ULTRA: ---")

        self.root.after(250, self.update_all)

    def chiudi_app(self):
        """Funzione chiamata dal tasto ESCI"""
        print("Chiusura in corso...")
        GPIO.cleanup()
        self.root.destroy()
        sys.exit()

if __name__ == "__main__":
    root = tk.Tk()
    app = DistanceApp(root)
    # Gestisce anche la chiusura tramite la 'X' della finestra
    root.protocol("WM_DELETE_WINDOW", app.chiudi_app)
    root.mainloop()
