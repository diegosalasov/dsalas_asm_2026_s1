import serial
import time
import numpy as np
import librosa

# --- CONFIGURACIÓN ---
N = 256  # Tamaño de los datos
PUERTO = 'COM12'
BAUD = 1000000
SAMPLERATE = 8000
HEADER = 0xAA
FOOTER = 0x55

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
        trama = bytearray([HEADER]) + bloque.tobytes() + bytearray([FOOTER])
        
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


# Carga el archivo con una frecuencia de muestreo específica (8000Hz)
# Librosa devuelve 'audio' como un array de floats entre -1.0 y 1.0
audio_raw, _ = librosa.load("Audio/cancion_pokemon.mp3", sr=SAMPLERATE, mono=True)

# Restamos el valor mínimo para que el punto más bajo sea exactamente 0
# Ahora todos los valores de la canción son positivos
audio_positivo = audio_raw - audio_raw.min()

# Dividimos por el nuevo máximo para que el rango sea de 0.0 a 1.0
audio_normalizado = audio_positivo / audio_positivo.max()

# Multiplicamos por 150 para definir el "techo" de volumen (de 255 posibles)
audio_escalado = audio_normalizado * 150

# Convertimos de float a unsigned integer de 8 bits (0-255)
# Esto es lo que finalmente viajará al ESP32
data = audio_escalado.astype(np.uint8)


transmitir(data, N)