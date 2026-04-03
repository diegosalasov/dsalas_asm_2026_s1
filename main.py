import serial
import time
import numpy as np
import librosa
import os

# --- CONFIGURACIÓN ---
N = 128  # 2^7 (Cumple con potencia de 2)
PUERTO = 'COM12'
BAUD = 115200
SAMPLING_RATE = 8000

def transmitir_128(muestras):
    # Inicializar Serial
    ser = serial.Serial(PUERTO, BAUD, timeout=1)
    time.sleep(3) # Esperar reinicio
    ser.reset_input_buffer()
    
    # Sincronización Inicial
    print("Esperando 'S' del ESP32...")
    while True:
        if ser.in_waiting > 0:
            if ser.read() == b'S':
                ser.write(b'A')
                break
    
    print("Sincronizado. Enviando canción en bloques de 128...")
    total = len(muestras)
    
    # Enviar en bloques de 128 muestras
    for i in range(0, total, N):
        bloque = muestras[i : i + N]
        if len(bloque) < N:
            bloque = np.pad(bloque, (0, N - len(bloque)), 'constant', constant_values=127)
        
        # Enviar bloque
        ser.write(bloque.tobytes())
        ser.flush()

        # Esperar la 'K' con un pequeño margen
        timeout_at = time.time() + 2
        confirmado = False
        while time.time() < timeout_at:
            if ser.in_waiting > 0:
                if ser.read() == b'K':
                    confirmado = True
                    break
        
        if not confirmado:
            print(f"\n Error en bloque {i//N}")
            return

        print(f"Progreso: {(i/total)*100:.2f}%", end='\r')
    
    ser.close()
    print("\n Canción enviada con éxito.")

# Cargar y normalizar
data, _ = librosa.load("Audio/cancion_la.mp3", SAMPLING_RATE, mono=True)
data = ((data - data.min()) / (data.max() - data.min()) * 255).astype(np.uint8)

transmitir_128(data)