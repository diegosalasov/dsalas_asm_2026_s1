#include <Arduino.h>
#include "arduinoFFT.h"

/* --- CONSTANTES DE COMUNICACIÓN --- */
#define N 256                 // Tamaño de cada bloque de datos recibido
#define N_FFT 256             // Tamaño total del buffer para procesamiento (FFT/Reproducción)
#define HEADER 0xAA           // Marcador de inicio de trama
#define FOOTER 0x55           // Marcador de fin de trama
#define SAMPLE_PERIOD 125     // Periodo de muestreo en microsegundos (para 8000 Hz)
#define BAUD_RATE_PC 1000000       // Velocidad de comunicación serial (puede ajustarse según estabilidad)
#define SERIAL_2_BAUD 115200

/* --- CONSTANTES DE CONTROL --- */
#define ACK_SIGNAL 'K'   // Señal de ACK para la comunicación entre tarjetas (puedes cambiarla si quieres)
#define GET 'G'          // Señal para solicitar un bloque a Python
#define ERROR 'E'        // Señal para indicar un error en la trama
#define START 'S'        // Señal para iniciar la sincronización con Python
#define ACKNOWLEDGE 'A'  // Señal para indicar que la sincronización fue exitosa

/* --- VARIABLES GLOBALES --- */
uint8_t buffer[N];            // Almacena temporalmente los datos de una trama
uint8_t FFT_buffer[N_FFT];    // Buffer acumulador para procesamiento de 512 muestras
int block_counter = 0;        // Contador de bloques N recibidos



/* --- VARIABLES FFT --- */
float vReal[N_FFT];
float vImag[N_FFT];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N_FFT, 8000);

struct FrequencyGroup {
    uint16_t index;     // Índice 'k' de la FFT
    float energy;      // Energía del par (k y N-k)
};

void setup() {
  // Aumentamos el buffer de hardware de la UART para evitar desbordamientos
  Serial.setRxBufferSize(2048); 
  Serial.begin(BAUD_RATE_PC);
  

  // Configuración del Buffer de recepción para el Handshake
  Serial2.setRxBufferSize(1024); 

  // Inicialización de UART2: 
  // Velocidad: 1 Mbps (puedes bajarla a 115200 si falla)
  // Protocolo: SERIAL_8N1 (8 bits, sin paridad, 1 stop)
  // Pines: RX=16, TX=17
  Serial2.begin(SERIAL_2_BAUD, SERIAL_8N1, 16, 17);

  
  pinMode(25, OUTPUT);        // Pin del DAC interno del ESP32

  // --- HANDSHAKE INICIAL ---
  // El ESP32 espera recibir 'S' para confirmar que Python está listo
  while (true) {
    if (Serial.available() > 0 && Serial.read() == START) {
      Serial.write(ACKNOWLEDGE);      // Responde 'A' (Acknowledge) para iniciar flujo
      break;
    }
  }
}

void loop() {
  // 1. SOLICITAR BLOQUE A PYTHON
  // Enviamos 'G' para que Python nos mande UNA trama (Header + 256 bytes + Footer)
  Serial.write(GET);

  // 2. ESPERAR HASTA QUE LLEGUE LA TRAMA COMPLETA (258 bytes)
  // Usamos un pequeño timeout para que no se quede colgado si Python falla
  uint32_t t_espera = millis();
  while (Serial.available() < (N + 2)) {
    if (millis() - t_espera > 500) {
      Serial.write(GET); // Re-solicitar si Python se tardó mucho
      t_espera = millis();
    }
    yield(); 
  }

  // 3. PROCESAR LA TRAMA RECIBIDA
  if (Serial.read() == HEADER) {
    
    // Leer los datos de audio
    Serial.readBytes(buffer, N);
    
    // Verificar que el cierre de trama sea correcto
    if (Serial.read() == FOOTER) {
      
      // Llenar buffer para FFT
      for (int i = 0; i < N; i++) {
        vReal[i] = (float)buffer[i];
        vImag[i] = 0.0;
      }

      // --- PROCESAMIENTO MATEMÁTICO ---
      FFT.compute(FFT_FORWARD);
      
      // Aquí podrías aplicar la compresión si la descomentas:
      // applyEnergyBasedCompression(vReal, vImag, N_FFT, 0.95);


      
      // --- COMUNICACIÓN CON TARJETA 2 ---
      sendFftBlock(); // Envía los floats procesados a la otra tarjeta

      // --- BLOQUEO POR HANDSHAKE (ESPERA A T2) ---
      // IMPORTANTE: No pedimos más música a Python hasta que la T2 confirme
      uint32_t t_hshake = millis();
      while (true) {
        if (Serial2.available() > 0) {
          if (Serial2.read() == ACK_SIGNAL) { 
            break; // La Tarjeta 2 ya terminó de procesar/reproducir
          }
        }
        // Si la Tarjeta 2 no responde en 200ms, seguimos para no trabar el sistema
        if (millis() - t_hshake > 200) break; 
        yield();
      }

      // --- LIMPIEZA Y REPETICIÓN ---
      // No necesitamos Serial.write('K') porque el nuevo 'G' al inicio del loop
      // es el que le sirve a Python como confirmación de "Dame más".
      block_counter = 0; 

    } else {
      // Si el footer falló, enviamos 'E' y el loop vuelve a pedir el bloque con 'G'
      Serial.write(ERROR);
      while(Serial.available()) Serial.read(); // Limpiar basura
    }
  } else {
    // Si el byte inicial no era HEADER, limpiar hasta encontrar uno o pedir de nuevo
    while(Serial.available() && Serial.peek() != HEADER) Serial.read();
  }
}


/**
 * Aplica compresión basada en la importancia energética de los componentes.
 * Basado en el algoritmo de preservación de energía de tu script de Python.
 */
void applyEnergyBasedCompression(double *vReal, double *vImag, uint16_t samples, double target_ratio) {
    uint16_t half_count = samples / 2;
    FrequencyGroup groups[half_count + 1];
    double total_energy = 0;

    // 1. Agrupar coeficientes y calcular energía total
    // Tratamos DC (0) y Nyquist (half_count) por separado, el resto en pares.
    for (uint16_t k = 0; k <= half_count; k++) {
        groups[k].index = k;
        if (k == 0 || k == half_count) {
            groups[k].energy = (vReal[k] * vReal[k]) + (vImag[k] * vImag[k]);
        } else {
            // Energía del par conjugado k y N-k
            double e_k = (vReal[k] * vReal[k]) + (vImag[k] * vImag[k]);
            double e_nk = (vReal[samples - k] * vReal[samples - k]) + (vImag[samples - k] * vImag[samples - k]);
            groups[k].energy = e_k + e_nk;
        }
        total_energy += groups[k].energy;
    }

    // 2. Ordenar grupos por energía (Descendente - Selection Sort)
    for (uint16_t i = 0; i <= half_count; i++) {
        uint16_t max_idx = i;
        for (uint16_t j = i + 1; j <= half_count; j++) {
            if (groups[j].energy > groups[max_idx].energy) max_idx = j;
        }
        FrequencyGroup temp = groups[i];
        groups[i] = groups[max_idx];
        groups[max_idx] = temp;
    }

    // 3. Identificar componentes necesarios para alcanzar el target_ratio
    double cumulative_energy = 0;
    uint16_t components_to_keep = 0;
    double energy_threshold = total_energy * target_ratio;

    bool mask[half_count + 1] = {false}; // Máscara para saber qué índices conservar

    for (uint16_t i = 0; i <= half_count; i++) {
        cumulative_energy += groups[i].energy;
        mask[groups[i].index] = true;
        components_to_keep++;
        if (cumulative_energy >= energy_threshold) break;
    }

    // 4. Comprimir: Poner a cero lo que no está en la máscara
    for (uint16_t k = 0; k <= half_count; k++) {
        if (!mask[k]) {
            vReal[k] = 0;
            vImag[k] = 0;
            if (k > 0 && k < half_count) {
                vReal[samples - k] = 0;
                vImag[samples - k] = 0;
            }
        }
    }
}

void sendFftBlock() {
    Serial2.write(HEADER);
    Serial2.write((uint8_t*)vReal, N_FFT * sizeof(float)); 
    Serial2.write((uint8_t*)vImag, N_FFT * sizeof(float));
    Serial2.write(FOOTER);
    Serial2.flush(); // Asegura que los bytes salieron físicamente del chip
}