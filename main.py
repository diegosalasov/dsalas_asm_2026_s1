import serial
import time
import numpy as np
import librosa

# --- CONFIGURACIÓN ---
N = 256  # Tamaño de los datos
PUERTO = 'COM5'
BAUD = 1000000
SAMPLERATE = 8000
HEADER_DATA = 0xA1
HEADER_END  = 0xA2
FOOTER = 0x55

# --- FLAGS ---
GET = b'G'  # Orden del ESP32 para pedir un bloque
ACKNOWLEDGE = b'A' # Orden del ESP32 para indicar que el bloque fue recibido correctamente
START = b'S' # Orden del PC para iniciar la sincronización
ERROR = b'E' # Orden del ESP32 para indicar que hubo un error en la trama
OK = b'K' # Orden del ESP32 para indicar que la trama fue recibida correctamente
STAGE_1 = b'1' # Orden del PC para indicar que se va a enviar la etapa 1 (Metricas)
STAGE_2 = b'2' # Orden del PC para indicar que se va a enviar la etapa 2 (Reproducción)


def transmitir(muestras, tam_bloque):
    # Configuración del puerto
    # Nota: El timeout se pone en None para que espere pacientemente la orden del ESP
    ser = serial.Serial(PUERTO, BAUD, timeout=None) 
    time.sleep(2)  # Esperar a que el ESP32 se reinicie tras abrir el puerto
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    print("Sincronizando con Tarjeta 1...")
    # --- HANDSHAKE INICIAL ---
    while True:
        ser.write(START)
        if ser.read(1) == ACKNOWLEDGE:
            print("Sincronización exitosa. Esperando peticiones de bloques...")
            break
        time.sleep(0.1)

    total = len(muestras)
    num_bloques = int(np.ceil(total / tam_bloque))


    print("Enviando etapa 1...\n")
    sendblock(0, num_bloques, muestras, tam_bloque, ser, STAGE_1)

    time.sleep(1)  # Pequeña pausa entre etapas

    print("Enviando etapa 2...\n")
    sendblock(0, num_bloques, muestras, tam_bloque, ser, STAGE_2)
    

    ser.close()
    print("\n\nTransmisión finalizada con éxito.")


def sendblock(i, num_bloques, muestras, tam_bloque, ser, etapa):
    while i <= num_bloques:
        # 1. ESPERAR LA ORDEN 'G' (DAME BLOQUE) DEL ESP32
        # El script se detiene aquí hasta que la Tarjeta 1 esté lista
        orden = ser.read(1)
        
        if orden == GET:
            # Preparar el bloque actual
            ser.write(etapa)

            bloque : np.ndarray
            header = 0
            if (i == num_bloques):
                # Padding si es el ultimo bloque (datos irrelevantes)
                bloque = np.pad(bloque, (0, tam_bloque), 'constant', constant_values=127)
                header = HEADER_END
            else:
                inicio = i * tam_bloque
                bloque = muestras[inicio:inicio+tam_bloque]
                
                # Padding si es el último bloque (con datos)
                if len(bloque) < tam_bloque:
                    bloque = np.pad(bloque, (0, tam_bloque - len(bloque)), 'constant', constant_values=127)
                header = HEADER_DATA
                
            # --- EMPAQUETADO ---
            # Header (0xAA) + Datos + Footer (0x55)
            trama = bytearray([header]) + bloque.tobytes() + bytearray([FOOTER])
            
            # 2. ENVIAR LA TRAMA
            ser.write(trama)
            
            # Feedback de progreso
            if True:
                print(f"Enviando bloque {i}/{num_bloques} - Progreso: {(i/num_bloques)*100:.1f}%", end='\r')
            
            i += 1 # Avanzar al siguiente bloque solo después de enviarlo
        else:
            # Si llega cualquier otra cosa (ruido), simplemente seguimos esperando un 'G'
            continue
    return

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