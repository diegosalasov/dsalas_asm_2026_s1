#include <Arduino.h>

#define N 128 // 2^7
uint8_t buffer[N];

void setup() {
  Serial.begin(115200);
  
  // Fase de Sincronización
  while (true) {
    Serial.write('S');
    if (Serial.available() > 0) {
      if (Serial.read() == 'A') break;
    }
    delay(100);
  }
}

void loop() {
  // Si el buffer tiene al menos 128 bytes
  if (Serial.available() >= N) {
    // Leemos el bloque
    Serial.readBytes(buffer, N);
    
    // Aquí podrías añadir un dacWrite para probar el sonido:
    /*
    for(int i=0; i<N; i++) {
       dacWrite(25, buffer[i]);
       delayMicroseconds(125); 
    }
    */

    // Confirmar a Python para recibir el siguiente bloque de 128
    Serial.write('K'); 
  }
}