#include <Arduino.h>
#include "arduinoFFT.h"

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // Verificamos si hay suficientes bytes para un bloque completo de doubles
  // (N_FFT * sizeof(double) * 2) + Header + Footer
  if (Serial2.available() >= ((N_FFT * 16) + 2)) {
    
    if (Serial2.read() == HEADER) {
      // 1. LEER LOS DATOS DE LA FFT (vReal y vImag)
      Serial2.readBytes((char*)vReal, N_FFT * sizeof(double));
      Serial2.readBytes((char*)vImag, N_FFT * sizeof(double));
      
      if (Serial2.read() == FOOTER) {
        
        // 2. COMPRESIÓN (Opcional en esta tarjeta)
        applyEnergyBasedCompression(vReal, vImag, N_FFT, 0.95);

        // 3. IFFT (Regreso al tiempo)
        FFT.compute(FFT_REVERSE);

        // 4. REPRODUCCIÓN (Escalado 1/N)
        for (int i = 0; i < N_FFT; i++) {
          uint32_t t_inicio = micros();
          
          double sample = vReal[i] / N_FFT; // Normalización indispensable
          
          if (sample > 255) sample = 255;
          if (sample < 0)   sample = 0;

          dacWrite(25, (uint8_t)sample);
          
          while ((micros() - t_inicio) < SAMPLE_PERIOD);
        }
      }
    }
  }
} 