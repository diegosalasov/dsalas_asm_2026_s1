#include <Arduino.h>
#include <LiquidCrystal.h>
#include "arduinoFFT.h"
#include <math.h>
#include <ESP32DMASPISlave.h> // Usamos la versión Slave para recibir

// --- Configuración de Protocolo ---
#define N_FFT          256
#define HEADER_A       0xAA     // Marcador de inicio de trama (bloque FFT)
#define HEADER_B       0xAB     // Marcador de inicio de trama (bloque onda)
#define HEADER_END     0xA2     // Marcador de inicio de trama (final de audio)
#define FOOTER         0x55     // Marcador de fin de trama
#define SAMPLE_PERIOD  125      // 8000 Hz -> 125us por muestra

// --- Configuración SPI DMA (VSPI nativo) ---
#define SPI_MISO      19
#define SPI_MOSI      23
#define SPI_SCLK      18
#define SPI_SS        5

// --- Definicion de pines LCD
#define DB7 26
#define DB6 25
#define DB5 33
#define DB4 32
#define E   21
#define RS  22

// Cálculo del Buffer: 2050 datos + 2 alineación + 4 compensación bug = 2056
#define SPI_BUFFER_SIZE 2056

// --- Variables y Objetos ---
float vReal[N_FFT];
float vImag[N_FFT];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N_FFT, 8000);

ESP32DMASPI::Slave slave;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

float vReal2[N_FFT];
float vImag2[N_FFT];
LiquidCrystal lcd(RS,E,DB4,DB5,DB6,DB7);

bool state[] = {false, false};
bool _lcd_init = false;

int samples = N_FFT;
float signal_diff2 = 0.0f;
float signal_orig2 = 0.0f;
float signal_new2  = 0.0f;
float MSE = 0.0f;
float SER = 0.0f;
float E_preserved = 0.0f; 
int block_count = 0;

void setup() {
  // Inicializar Serial solo para depuración (opcional)
  Serial.begin(115200);

  // --- Configuración de Buffers DMA ---
  dma_tx_buf = slave.allocDMABuffer(SPI_BUFFER_SIZE);
  dma_rx_buf = slave.allocDMABuffer(SPI_BUFFER_SIZE);
  
  // Limpiar buffers
  memset(dma_tx_buf, 0, SPI_BUFFER_SIZE);
  memset(dma_rx_buf, 0, SPI_BUFFER_SIZE);

  // --- Configuración del Esclavo SPI ---
  // Según los "Known Issues", SPI_MODE1 es el más estable para evitar bit-shift
  slave.setDataMode(SPI_MODE1);
  slave.setMaxTransferSize(SPI_BUFFER_SIZE);
  slave.setQueueSize(1);
  
  // Iniciar con pines nativos VSPI
  // slave.begin(VSPI_HOST, SCK, MISO, MOSI, SS)
  slave.begin(VSPI_HOST, SPI_SCLK, SPI_MISO, SPI_MOSI, SPI_SS);

  Serial.printf("Esclavo SPI DMA VSPI Iniciado \n");
  
  // --- Configuracion de pantalla LCD ---
  lcd.begin(16,4);
  lcd.setCursor(0,0);
  lcd.print("Inicializando...");
}

void loop() {
  // 1. Esperar y recibir la transferencia por DMA
  // Esta función bloquea hasta que el Maestro completa el envío de SPI_BUFFER_SIZE bytes
  uint32_t received_bytes = slave.transfer(dma_tx_buf, dma_rx_buf, SPI_BUFFER_SIZE);
  if (received_bytes > 0) {
    // 2. Verificar sincronía (Header y Footer)
    // El footer está en el índice 2049 (1 + 1024 + 1024)
    if (dma_rx_buf[0] == HEADER_A && dma_rx_buf[1 + sizeof(vReal) + sizeof(vImag)] == FOOTER) { // Bloques FFT
        // 3. Desempaquetar los floats del buffer DMA a los arrays de la FFT
        memcpy(vReal, &dma_rx_buf[1], sizeof(vReal));
        memcpy(vImag, &dma_rx_buf[1 + sizeof(vReal)], sizeof(vImag));

        // 4. Procesamiento: Transformada Inversa (Frecuencia -> Tiempo)
        FFT.compute(FFT_REVERSE);
        
        // --- Activar bandera de recepcion de un bloque FFT ---
        state[1] = true;
        block_count += 1;
    } else if (dma_rx_buf[0] == HEADER_B && dma_rx_buf[1 + sizeof(vReal) + sizeof(vImag)] == FOOTER) { // Bloques onda
        // 3. Desempaquetar los floats del buffer DMA a los arrays
        memcpy(vReal2, &dma_rx_buf[1], sizeof(vReal));
        memcpy(vImag2, &dma_rx_buf[1 + sizeof(vReal)], sizeof(vImag));
        
        // --- Activar bandera de recepcion de un bloque onda ---
        state[0] = true;
        block_count += 1;
    } else if (dma_rx_buf[0] == HEADER_END && dma_rx_buf[1 + sizeof(vReal) + sizeof(vImag)] == FOOTER) { // Final de audio
        // 3, Mostrar metricas
        
    } else {
        // En caso de error de sincronía, limpiamos el buffer RX
        memset(dma_rx_buf, 0, SPI_BUFFER_SIZE);
    }

  }

  // 5. Verificar si se han recibido ambos bloques para iniciar calculo de metricas
  if (state[0] && state[1]) updateMetrics();
}

void updateMetrics () {
    // 1. Recacular metricas para la cantidad de muestras recibidas
    // cada bloque aporta 256 muestras

    // 2. Procesar datos de ambas ondas
    float so, sn, diff;
    for (int n = 0; n < N_FFT; n++){
        // Calcular magnitud en cada punto para contemplar pequenas variaciones en la parte imaginaria
        so = sqrt( pow(vReal2[n], 2) + pow(vImag2[n], 2) );
        sn = sqrt( pow(vReal[n], 2)  + pow(vImag[n], 2)  );

        // Calcular valores acumulados
        signal_orig2 += so * so; // x[n]^2
        signal_new2  += sn * sn; // x'[x]^2
        
        diff = so - sn;
        signal_diff2 += diff * diff; // (x[n] - x-[n])^2
    }

    // 3. Actualizar ambas metricas utilizando los valores acumulados de las ondas
    MSE = signal_diff2 / samples;      
    SER = 10 * log10( signal_orig2 / signal_diff2 );
    E_preserved = signal_new2 / signal_orig2 * 100; // valor escalado en 100 (porcentaje)
    
    // 4. Reiniciar acumuladores
    signal_orig2 = 0.0f;
    signal_new2  = 0.0f;
    signal_diff2 = 0.0f;

    // 5. Resetear banderas
    state[0] = false; state[1] = false;
    displayMetrics();
}

void displayMetrics () {
    // Limpiar valores de pantalla
    if (!_lcd_init){ // --- Inicializar nombres y valores de variables (solo una vez) ---
        _lcd_init = true; // Actualizar estado
        lcd.clear();
        // 0. Encabezado
        lcd.setCursor(0, 0);
        lcd.print("--- Metricas ---");

        // 1. Escribir variable de MSE (error cuadratico medio)
        lcd.setCursor(0, 1);
        lcd.print("MSE = ");
        lcd.print(MSE,2);

        // 2. Escribir variable Ep (energia preservada)
        lcd.setCursor(0, 2);
        lcd.print("Ep  = ");
        lcd.print(E_preserved,2);

        // 3. Escribir variable SER (relacion senal a error)
        lcd.setCursor(0, 3);
        lcd.print("SER = ");
        lcd.print(SER,2);

    } else {         // --- Limpiar contenidos y escribir nuevos valores de metricas ---
        // 1. Escribir valor MSE
        lcd.setCursor(6,1);
        lcd.print("          ");
        lcd.setCursor(6,1);
        lcd.print(MSE,2);

        // 2. Escribir valor Ep
        lcd.setCursor(6,2);
        lcd.print("          ");
        lcd.setCursor(6,2);
        lcd.print(E_preserved,2);

        // 3. Escribir valor SER
        lcd.setCursor(6,3);
        lcd.print("          ");
        lcd.setCursor(6,3);
        lcd.print(SER,2);
    }
    // (opcional) Imprimir los valores por serial
    /* Descomentar estas lineas */
    // Serial.printf("MSE = %.2f \n", MSE);
    // Serial.printf("Ep  = %.2f \n", E_preserved);
    // Serial.printf("SER = %.2f \n", SER);
}
