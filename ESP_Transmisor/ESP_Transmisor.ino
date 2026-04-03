#include <Arduino.h>

/* --- CONSTANTES DE COMUNICACIÓN --- */
#define N 128                 // Tamaño de cada bloque de datos recibido
#define N_FFT 512             // Tamaño total del buffer para procesamiento (FFT/Reproducción)
#define HEADER 0xAA           // Marcador de inicio de trama
#define FOOTER 0x55           // Marcador de fin de trama
#define SAMPLE_PERIOD 120     // Periodo de muestreo en microsegundos (para 8000 Hz)

/* --- VARIABLES GLOBALES --- */
uint8_t buffer[N];            // Almacena temporalmente los datos de una trama
uint8_t FFT_buffer[N_FFT];    // Buffer acumulador para procesamiento de 512 muestras
int block_counter = 0;        // Contador de bloques N recibidos (0 a 3 para N=128)

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
        
        // Trama válida: Informar a Python (ACK) para que envíe el siguiente bloque
        Serial.write('K'); 

        // Copiar el bloque actual a la posición correspondiente en el buffer grande
        for (int i = 0; i < N; i++) {
          FFT_buffer[i + (N * block_counter)] = buffer[i];
        }
        block_counter++;

        /* * PROCESAMIENTO Y REPRODUCCIÓN:
         * Una vez que el FFT_buffer está lleno (4 bloques de 128 = 512 muestras)
         */
        if (block_counter >= (N_FFT / N)) {
          
          for (int i = 0; i < N_FFT; i++) {
            uint32_t t_inicio = micros();
            
            // Salida analógica al DAC (Pin 25)
            dacWrite(25, FFT_buffer[i]);
            
            // Asegura un intervalo exacto de 125us entre muestras (8kHz)
            while ((micros() - t_inicio) < SAMPLE_PERIOD);
          }
          
          // Reiniciar contador para recibir los siguientes 512 bytes
          block_counter = 0;

          // Si hay basura acumulada en el buffer serial (>512 bytes), se descarta 
          // para mantener la latencia baja y evitar desincronía
          if (Serial.available() > 512) {
            while(Serial.available() > 0) Serial.read();
          }
        }
      } else {
        // ERROR DE FOOTER: Indica que se perdió la alineación de bytes
        Serial.write('E'); 
      }
    }
    // Si no es el HEADER, el loop descarta el byte y vuelve a buscar en la siguiente iteración
  }
}