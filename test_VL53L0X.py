import time
import board
import busio
import adafruit_vl53l0x

# Inizializzazione del bus I2C (Pin 3 SDA, Pin 5 SCL)
i2c = busio.I2C(board.SCL, board.SDA)

# Inizializzazione del sensore
# L'indirizzo predefinito è 0x29
try:
    vl53 = adafruit_vl53l0x.VL53L0X(i2c)
    print("VL53L0X inizializzato correttamente!")
except ValueError:
    print("Sensore non trovato sul bus I2C. Controlla i cablaggi.")
    exit()

print("Avvio letture in corso... Premi CTRL+C per interrompere.")
print("-" * 30)

try:
    while True:
        # La proprietà 'range' restituisce la distanza in millimetri
        distanza = vl53.range
        
        print(f"Distanza: {distanza} mm")
        
        # Un piccolo delay per non sovraccaricare la CPU
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLettura interrotta dall'utente.")
