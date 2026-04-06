#include <Arduino.h>
#include "arduinoFFT.h"

/* --- CONSTANTES DE COMUNICACIÓN --- */
#define N 256                 // Tamaño de cada bloque de datos recibido
#define N_FFT 256             // Tamaño total del buffer para procesamiento (FFT/Reproducción)
#define HEADER 0xAA           // Marcador de inicio de trama
#define FOOTER 0x55           // Marcador de fin de trama
#define SAMPLE_PERIOD 125     // Periodo de muestreo en microsegundos (para 8000 Hz)
#define BAUD_RATE_PC 1000000       // Velocidad de comunicación serial (puede ajustarse según estabilidad)
#define SERIAL_2_BAUD 250000

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


struct component {
  float energy;
  uint16_t index;
  bool conservate;
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
      compressFft(N, 0.95); 

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

      /*
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
      
      
      */

    
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




void sendFftBlock() {
    Serial2.write(HEADER);
    Serial2.write((uint8_t*)vReal, N_FFT * sizeof(float)); 
    Serial2.write((uint8_t*)vImag, N_FFT * sizeof(float));
    Serial2.write(FOOTER);
    Serial2.flush(); // Asegura que los bytes salieron físicamente del chip
}