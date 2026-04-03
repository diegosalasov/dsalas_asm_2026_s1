#include <Arduino.h>

const uint8_t HEADER = 0xAA;
const uint8_t FOOTER = 0x55;
const char ACK_SIGNAL = 'K';

void setup() {
  // IMPORTANTE: El buffer debe ser lo suficientemente grande para 2 tramas
  Serial2.setRxBufferSize(4096); 
  Serial2.begin(115200, SERIAL_8N1, 16, 17);

}

// Tarjeta 2 - Loop optimizado
void loop() {
  // Solo entrar si hay al menos una trama completa esperando
  if (Serial2.available() >= 2050) {
    
    if (Serial2.read() == HEADER) {
      uint8_t dump[2048];
      // readBytes tiene un timeout interno, es más seguro
      size_t leidos = Serial2.readBytes(dump, 2048); 
      
      uint8_t footer = Serial2.read();
      
      if (footer == FOOTER && leidos == 2048) {
        // TRAMA PERFECTA
        Serial2.write(ACK_SIGNAL); 
      } else {
        // Si falló, limpiamos el buffer agresivamente para no arrastrar el error
        while(Serial2.available() > 0) Serial2.read();
      }
    }
  }
}