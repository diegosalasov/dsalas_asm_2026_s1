#include <Arduino.h>
#include "arduinoFFT.h"

/* --- CONSTANTES DE COMUNICACIÓN --- */
#define N 256                 // Tamaño de cada bloque de datos recibido
#define N_FFT 256             // Tamaño total del buffer para procesamiento (FFT/Reproducción)
#define HEADER 0xAA           // Marcador de inicio de trama
#define FOOTER 0x55           // Marcador de fin de trama
#define SAMPLE_PERIOD 125     // Periodo de muestreo en microsegundos (para 8000 Hz)

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
  Serial.begin(1000000);
  pinMode(25, OUTPUT);        // Pin del DAC interno del ESP32

  // --- HANDSHAKE INICIAL ---
  // El ESP32 espera recibir 'S' para confirmar que Python está listo
  while (true) {
    if (Serial.available() > 0 && Serial.read() == 'S') {
      Serial.write('A');      // Responde 'A' (Acknowledge) para iniciar flujo
      break;
    }
  }
}

void loop() {
  /* * RECEPCIÓN DE TRAMAS:
   * Se espera una trama con formato: [HEADER] + [N BYTES DE AUDIO] + [FOOTER]
   * Longitud total requerida: N + 2 bytes
   */
  if (Serial.available() >= (N + 2)) {
    
    // 1. BUSCAR HEADER: Sincroniza el inicio del bloque
    if (Serial.read() == HEADER) {
      
      // 2. LEER DATOS: Extrae los bytes de audio del flujo serial
      Serial.readBytes(buffer, N);
      
      // 3. VERIFICAR FOOTER: Valida que la trama llegó íntegra y sin desfases
      if (Serial.read() == FOOTER) {
        Serial.write('K'); 

        // Llenar buffer de 512
        for (int i = 0; i < N; i++) {
          FFT_buffer[i + (N * block_counter)] = buffer[i];
        }
        block_counter++;

        // PROCESAMIENTO CUANDO EL BUFFER ESTÁ LLENO
        if (block_counter >= (N_FFT / N)) {
          
          // Preparar datos (Convertir uint8 a double y limpiar imaginarios)
          for (int i = 0; i < N_FFT; i++) {
            vReal[i] = (float)FFT_buffer[i];
            vImag[i] = 0.0;
          }

          // Ejecutar FFT (Dominio del Tiempo -> Frecuencia)
          FFT.compute(FFT_FORWARD);
          
          // applyEnergyBasedCompression(vReal, vImag, N_FFT, 0.95); // Mantener el 90% de la energía

          // Ejecutar FFT (Dominio del Frecuencia -> Frecuencia)
          FFT.compute(FFT_REVERSE);

          // REPRODUCCIÓN DE LA SEÑAL RECONSTRUIDA
          for (int i = 0; i < N_FFT; i++) {
            uint32_t t_inicio = micros();
            
            // Casting de double a uint8 para el DAC
            // Nota: La IFFT puede requerir normalización si los valores escalan
            dacWrite(25, (uint8_t)vReal[i]);
            
            while ((micros() - t_inicio) < SAMPLE_PERIOD);
          }
          
          block_counter = 0;
          if (Serial.available() > 512) while(Serial.available() > 0) Serial.read();
        }
      } else {
        Serial.write('E'); 
      }
    }
    // Si no es el HEADER, el loop descarta el byte y vuelve a buscar en la siguiente iteración
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