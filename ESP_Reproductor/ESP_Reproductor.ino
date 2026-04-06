#include <Arduino.h>
#include "arduinoFFT.h"

#define N_FFT 256
#define HEADER 0xAA
#define FOOTER 0x55
#define SAMPLE_PERIOD 125 // 8000 Hz -> 1/8000 = 125us
#define ACK_SIGNAL 'K'
#define SERIAL_2_BAUD 250000

// Buffers para la IFFT
float vReal[N_FFT];
float vImag[N_FFT];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N_FFT, 8000);

void setup() {
  // Buffer grande para recibir los 2050 bytes sin pérdidas
  Serial2.setRxBufferSize(2560); 
  Serial2.begin(SERIAL_2_BAUD, SERIAL_8N1, 16, 17);
  
  // Pin 25 es el DAC1 interno del ESP32
  pinMode(25, OUTPUT); 
}

void loop() {
  // Esperar la trama completa: Header(1) + vReal(1024) + vImag(1024) + Footer(1) = 2050 bytes
  if (Serial2.available() >= 2050) {
    
    if (Serial2.read() == HEADER) {
      
      // Leemos los floats directamente a los arrays de la FFT
      // vReal y vImag ocupan 1024 bytes cada uno (256 * 4 bytes)
      Serial2.readBytes((uint8_t*)vReal, N_FFT * sizeof(float));
      Serial2.readBytes((uint8_t*)vImag, N_FFT * sizeof(float));
      
      if (Serial2.read() == FOOTER) {
        // 1. Avisar de inmediato a la Tarjeta 1 para que pida más bloques a Python
        Serial2.write(ACK_SIGNAL); 

        // 2. Transformada Inversa: Frecuencia -> Tiempo
        FFT.compute(FFT_REVERSE);

        // 3. Reproducción por el DAC
        // Recorremos los 256 valores reconstruidos
        for (int i = 0; i < N_FFT; i++) {
          uint32_t t_inicio = micros();
          
          // El resultado de la IFFT puede tener valores fuera de 0-255
          // Hacemos un cast simple, pero si escuchas ruido, podrías normalizarlo
          dacWrite(25, (uint8_t)vReal[i]);
          
          // Mantener el sample rate de 8000Hz (125 microsegundos por muestra)
          while ((micros() - t_inicio) < SAMPLE_PERIOD);
        }
        
      } else {
        // Error de sincronía: Limpiar buffer
        while(Serial2.available() > 0) Serial2.read();
      }
    }
  }
}

