import serial
import time
import numpy as np
import librosa

# --- CONFIGURACIÓN ---
N = 128  # Tamaño de los datos
PUERTO = 'COM12'
BAUD = 1000000

def transmitir(muestras, tam_bloque):
    ser = serial.Serial(PUERTO, BAUD, timeout=0.5)
    time.sleep(2)
    ser.reset_input_buffer()
    
    print("Sincronizando...")
    while True:
        ser.write(b'S')
        if ser.read(1) == b'A':
            break
        time.sleep(0.1)

    total = len(muestras)
    num_bloques = int(np.ceil(total / tam_bloque))
    
    for i in range(num_bloques):
        inicio = i * tam_bloque
        bloque = muestras[inicio:inicio+tam_bloque]
        
        if len(bloque) < tam_bloque:
            bloque = np.pad(bloque, (0, tam_bloque - len(bloque)), 'constant', constant_values=127)
            
        # --- EMPAQUETADO ---
        # Header (0xAA) + Datos + Footer (0x55)
        trama = bytearray([0xAA]) + bloque.tobytes() + bytearray([0x55])
        
        ser.write(trama)
        
        # Esperar la confirmación del ESP32
        confirmacion = ser.read(1)
        if confirmacion != b'K':
            # Si el ESP32 manda 'E' (Error), reintentamos o notificamos
            print(f"\nError de trama en bloque {i}")
            
        if i % 20 == 0:
            print(f"Progreso: {(i/num_bloques)*100:.1f}%", end='\r')

    ser.close()
    print("\nTransmisión exitosa.")

# Cargar y normalizar (Volumen a 150 para evitar golpeteo)
data, _ = librosa.load("Audio/cancion_pokemon.mp3", sr=8000, mono=True)
data = ((data - data.min()) / (data.max() - data.min()) * 150).astype(np.uint8)

transmitir(data, N)