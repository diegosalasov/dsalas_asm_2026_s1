#include <Arduino.h>
#include "arduinoFFT.h"
#include <ESP32DMASPIMaster.h>

/* --- CONSTANTES DE COMUNICACIÓN --- */
#define N 256                 // Tamaño de cada bloque de datos recibido
#define N_FFT 256             // Tamaño total del buffer para procesamiento (FFT/Reproducción)
#define HEADER_DATA 0xA1      // Marcador de inicio de trama para bloques de datos (serial)
#define HEADER_END  0xA2      // Marcador de inicio de trama para bloques de finalizacion (serial)
#define HEADER_A 0xAA         // Marcador de inicio de trama (bloques FFT)
#define HEADER_B 0xAB         // Marcador de inicio de trama (bloques onda)
#define FOOTER 0x55           // Marcador de fin de trama
#define SAMPLE_PERIOD 125     // Periodo de muestreo en microsegundos (para 8000 Hz)
#define BAUD_RATE_PC 1000000  // Velocidad de comunicación serial (puede ajustarse según estabilidad

/* --- CONSTANTES DE CONTROL --- */
#define ACK_SIGNAL 'K'   // Señal de ACK para la comunicación entre tarjetas (puedes cambiarla si quieres)
#define GET 'G'          // Señal para solicitar un bloque a Python
#define ERROR 'E'        // Señal para indicar un error en la trama
#define START 'S'        // Señal para iniciar la sincronización con Python
#define ACKNOWLEDGE 'A'  // Señal para indicar que la sincronización fue exitosa
#define STAGE_1 '1'
#define STAGE_2 '2'


/* --- VARIABLES GLOBALES --- */
uint8_t buffer[N];            // Almacena temporalmente los datos de una trama
uint8_t FFT_buffer[N_FFT];    // Buffer acumulador para procesamiento de 512 muestras
int block_counter = 0;        // Contador de bloques N recibidos
bool _setup_done = false;     // Bandera de setup: el setup consiste enviar todos los datos 
                              // al receptor 2 para calcular las metricas previo a iniciar reproduccion

     

/* --- VARIABLES FFT --- */
float vReal[N_FFT];
float vImag[N_FFT];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N_FFT, 8000);


struct component {
  float energy;
  uint16_t index;
  bool conservate;
};

// --- Definición de Pines SPI (VSPI nativo)---
#define SPI_MISO      19
#define SPI_MOSI      23
#define SPI_SCLK      18
#define SPI_CS_1      32
#define SPI_CS_2      33
#define SPEAKER_AVAIBLE 26


// --- Definición de Pines para leds ---
#define LED_1 25
#define LED_2 27

// 2056 bytes garantiza:
// 1. Alineación de 4 bytes para el DMA.
// 2. Que los 4 bytes que el esclavo "pierde" sean bytes vacíos al final.
#define SPI_BUFFER_SIZE  2056

/// --- Configuración del SPI ---
#define SPI_FREQUENCY 5000000 // 8 MHz, puedes ajustar según estabilidad
// (256 * 4) + (256 * 4) + 1 (Header) + 1 (Footer) + 4 bytes (bug) = 2054 
#define SPI_BUFFER_SIZE  2056

// --- Recursos DMA ---
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

// Instancia (cambiar a Slave si es el caso)
ESP32DMASPI::Master master;


void setup() {
  // to use DMA buffer, use these methods to allocate buffer
  dma_tx_buf = master.allocDMABuffer(SPI_BUFFER_SIZE);
  dma_rx_buf = master.allocDMABuffer(SPI_BUFFER_SIZE);

  master.setDataMode(SPI_MODE0);
  master.setFrequency(1000000);       
  master.setMaxTransferSize(SPI_BUFFER_SIZE); 
  master.begin(VSPI_HOST, SPI_SCLK, SPI_MISO, SPI_MOSI, -1);

  // Configuracion de pines
  pinMode(SPI_CS_1, OUTPUT);
  pinMode(SPI_CS_2, OUTPUT);

  digitalWrite(SPI_CS_1, HIGH);
  digitalWrite(SPI_CS_2, HIGH);

  pinMode(LED_1, OUTPUT);
  pinMode(LED_2, OUTPUT);

  pinMode(SPEAKER_AVAIBLE, INPUT_PULLDOWN);

  digitalWrite(LED_1, HIGH);
  digitalWrite(LED_2, HIGH);

  // Aumentamos el buffer de hardware de la UART para evitar desbordamientos
  Serial.setRxBufferSize(2048); 
  Serial.begin(BAUD_RATE_PC);
  

  // --- HANDSHAKE INICIAL ---
  // El ESP32 espera recibir 'S' para confirmar que Python está listo
  while (true) {
    if (Serial.available() > 0 && Serial.read() == START) {
      Serial.write(ACKNOWLEDGE);      // Responde 'A' (Acknowledge) para iniciar flujo
      break;
    }
  }

  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
}

void loop() {
  // 1. SOLICITAR BLOQUE A PYTHON
  while(Serial.available() > 0) Serial.read(); 

  // 1. SOLICITAR BLOQUE A PYTHON
  Serial.write(GET);

  // 2. ESPERAR CON TIMEOUT MEJORADO
  uint32_t t_espera = millis();
  while (Serial.available() < (N + 2)) {
    if (millis() - t_espera > 1000) { // Sube a 1s para ser más tolerante
      // En lugar de enviar otro GET y arriesgarte a duplicar, 
      // mejor limpia y reinicia el loop
      while(Serial.available() > 0) Serial.read();
      Serial.write(GET); 
      t_espera = millis();
    }
    yield(); 
  }
  
  // --- Recibir confirmacion de etapa ---
  char stage = Serial.read();
  if (stage == STAGE_1) _setup_done = false;
  if (stage == STAGE_2) _setup_done = true;

  // 3. PROCESAR LA TRAMA RECIBIDA
  int header = Serial.read();
  if (header == HEADER_DATA) {  // --- Lectura de datos de audio ---
    // Leer los datos de audio
    Serial.readBytes(buffer, N);
    
    // Verificar que el cierre de trama sea correcto
    if (Serial.read() == FOOTER) {
      // Llenar buffer para FFT
      for (int i = 0; i < N; i++) {
        vReal[i] = (float)buffer[i];
        vImag[i] = 0.0;
      }

      if (_setup_done) {// ETAPA REPRODUCCION (2): Reproduccion de audio en receptor 1
        // --- PROCESAMIENTO MATEMÁTICO ---
        FFT.compute(FFT_FORWARD);
      
        // --- APLICAR COMPRESION ---
        compressFft(N, 1); 

        // --- Esperar hasta que el esclavo esté disponible ---

        while (digitalRead(SPEAKER_AVAIBLE) == LOW) {
            delayMicroseconds(1); 
        }

        // Una vez que el pin es HIGH, el código continúa hacia abajo
        // Enviar FFT comprimida
        sendBlock(SPI_CS_1, HEADER_A);
      } else {          // ETAPA SETUP (1): Calculo de metricas en receptor 2
        // Enviar onda original
        sendBlock(SPI_CS_2, HEADER_B);

        // --- PROCESAMIENTO MATEMÁTICO ---
        FFT.compute(FFT_FORWARD);
      
        // --- APLICAR COMPRESION ---
        compressFft(N, 0.99);

        // Enviar FFT comprimida
        sendBlock(SPI_CS_2, HEADER_A);
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

  } else if (header == HEADER_END) {  // --- Lectura de cierre de datos de audio ---
    // Leer los datos de audio
    Serial.readBytes(buffer, N);
    
    // Verificar que el cierre de trama sea correcto
    if (Serial.read() == FOOTER) {
      // Llenar buffer
      for (int i = 0; i < N; i++) {
        vReal[i] = (float)buffer[i];
        vImag[i] = 0.0;
      }

      if (!_setup_done){ // ETAPA SETUP (1): Enviar cierre para calculo de metricas
        sendBlock(SPI_CS_2, HEADER_END);
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
    while(Serial.available() && Serial.peek() != HEADER_A) Serial.read();
  }

}


void compressFft(uint16_t samples, float target_ratio) {
  float total_energy = 0.0;
  uint16_t nyquist_idx = samples / 2; // 128

  float dc_energy = vReal[0] * vReal[0] + vImag[0] * vImag[0];
  float nyquist_energy = vReal[nyquist_idx] * vReal[nyquist_idx] + vImag[nyquist_idx] * vImag[nyquist_idx];

  // total_energy += dc_energy; // DC component
  // total_energy += nyquist_energy; // Nyquist component

  component component_witout_conjugate[129]; // Solo necesitamos la mitad de los componentes (sin conjugados)

  component_witout_conjugate[0] = {dc_energy, 0, true}; // DC siempre se conserva
  component_witout_conjugate[nyquist_idx] = {nyquist_energy, nyquist_idx, true}; // Nyquist también se conserva

  for (int i = 1; i < samples/2; i++) {
    total_energy += (vReal[i] * vReal[i] + vImag[i] * vImag[i]) * 2; // Contamos la energía de ambos componentes conjugados
    component_witout_conjugate[i].energy = (vReal[i] * vReal[i] + vImag[i] * vImag[i]) * 2; // Energía total de ambos componentes
    component_witout_conjugate[i].index = i;
    component_witout_conjugate[i].conservate = false; // Inicialmente no se conservan
  }

  quickSort(component_witout_conjugate, 1, nyquist_idx - 1); // Ordenamos por energía


  float energy_sum = 0.0;
  for (int i = 1; i <= nyquist_idx; i++) {
    energy_sum += component_witout_conjugate[i].energy;
    component_witout_conjugate[i].conservate = true; // Marcar para conservar
    if ((energy_sum / total_energy) >= target_ratio) {
      break; // Ya alcanzamos el ratio deseado
    }
  }

  // Recorremos TODO el array de componentes
  for (int i = 0; i <= nyquist_idx; i++) {
      // Si este componente NO fue marcado para conservar...
      if (!component_witout_conjugate[i].conservate) {
          int idx = component_witout_conjugate[i].index;
          // 1. Borramos el componente original
          vReal[idx] = 0.0;
          vImag[idx] = 0.0;
          
          // 2. Borramos la pareja conjugada SOLO si existe (no para DC o Nyquist)
          // El DC es idx=0 y Nyquist es idx=128
          if (idx > 0 && idx < nyquist_idx) {
              vReal[samples - idx] = 0.0;
              vImag[samples - idx] = 0.0;
          }
      }
  }

}



// Función para intercambiar dos elementos
void swap(component* a, component* b) {
    component t = *a; // Copia la estructura completa (energy + index + conservate)
    *a = *b;
    *b = t;
}

int partition(component arr[], int low, int high) {
    // Seleccionamos el último elemento como pivote
    int pivot = arr[high].energy;

    // Índice del elemento más pequeño
    int i = (low - 1);

    for (int j = low; j <= high - 1; j++) {
        // Si el elemento actual es menor o igual al pivote
        if (arr[j].energy > pivot) {
            i++;
            swap(&arr[i], &arr[j]);
        }
    }

    // Ponemos el pivote en su posición correcta
    swap(&arr[i + 1], &arr[high]);

    // Retornamos el punto de partición
    return (i + 1);
}

void quickSort(component arr[], int low, int high) {
    // Caso base: mientras el índice inicial sea menor al final
    if (low < high) {
        // pi es el índice de partición, arr[pi] ya está en su lugar
        int pi = partition(arr, low, high);

        // Ordenamos recursivamente antes y después de la partición
        quickSort(arr, low, pi - 1);
        quickSort(arr, pi + 1, high);
    }
}

void sendBlock(int CS, int header){
  int led = LED_1;
  if (CS == SPI_CS_2) led = LED_2;
  // 1. Configurar seleccion de slave para iniciar comunicacion
  digitalWrite(led, HIGH);
  digitalWrite(CS, LOW);
  // 2. Limpiar el buffer de transmisión para asegurar que el padding sea 0
  memset(dma_tx_buf, 0, SPI_BUFFER_SIZE);

  // 3. Insertar Marcador de Inicio (Header)
  dma_tx_buf[0] = header;

  // 4. Copiar vReal (256 * 4 bytes = 1024 bytes)
  // Destino: dma_tx_buf + 1
  memcpy(&dma_tx_buf[1], vReal, sizeof(vReal));

  // 5. Copiar vImag (256 * 4 bytes = 1024 bytes)
  // Destino: dma_tx_buf + 1 (header) + 1024 (vReal)
  memcpy(&dma_tx_buf[1 + sizeof(vReal)], vImag, sizeof(vImag));

  // 6. Insertar Marcador de Fin (Footer)
  // Posición: 1 + 1024 + 1024 = 2049
  dma_tx_buf[1 + sizeof(vReal) + sizeof(vImag)] = FOOTER;

  // 7. El resto del buffer (2050 a 2055) se queda como 0 (Padding)
  // Esto incluye los 4 bytes que el esclavo "perderá" por el bug del driver

  // 8. Iniciar transferencia DMA (Bloqueante en este caso)
  // Enviamos los 2056 bytes completos
  master.transfer(dma_tx_buf, dma_rx_buf, SPI_BUFFER_SIZE);

  // 9. Configurar seleccion de slave para detener comunicacion
  digitalWrite(CS, HIGH);
  digitalWrite(led, LOW);
}

/*
void sendfftBlock() {
    // 1. Limpiar el buffer de transmisión para asegurar que el padding sea 0
    memset(dma_tx_buf, 0, SPI_BUFFER_SIZE);

    // 2. Insertar Marcador de Inicio (Header)
    dma_tx_buf[0] = HEADER_A;

    // 3. Copiar vReal (256 * 4 bytes = 1024 bytes)
    // Destino: dma_tx_buf + 1
    memcpy(&dma_tx_buf[1], vReal, sizeof(vReal));

    // 4. Copiar vImag (256 * 4 bytes = 1024 bytes)
    // Destino: dma_tx_buf + 1 (header) + 1024 (vReal)
    memcpy(&dma_tx_buf[1 + sizeof(vReal)], vImag, sizeof(vImag));

    // 5. Insertar Marcador de Fin (Footer)
    // Posición: 1 + 1024 + 1024 = 2049
    dma_tx_buf[1 + sizeof(vReal) + sizeof(vImag)] = FOOTER;

    // 6. El resto del buffer (2050 a 2055) se queda como 0 (Padding)
    // Esto incluye los 4 bytes que el esclavo "perderá" por el bug del driver

    // 7. Iniciar transferencia DMA (Bloqueante en este caso)
    // Enviamos los 2056 bytes completos
    master.transfer(dma_tx_buf, dma_rx_buf, SPI_BUFFER_SIZE);
    delay(32);
} */