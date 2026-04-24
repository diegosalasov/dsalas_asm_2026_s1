#include <Arduino.h>
#include "arduinoFFT.h"
#include <ESP32DMASPISlave.h> // Usamos la versión Slave para recibir

// --- Configuración de Protocolo ---
#define N_FFT          256
#define HEADER         0xAA
#define FOOTER         0x55
#define SAMPLE_PERIOD  125      // 8000 Hz -> 125us por muestra

// --- Configuración SPI DMA (VSPI Nativo) ---
#define VSPI_MISO      19
#define VSPI_MOSI      23
#define VSPI_SCLK      18
#define VSPI_SS        13
#define SPEAKER_AVAIBLE 26

// Cálculo del Buffer: 2050 datos + 2 alineación + 4 compensación bug = 2056
#define SPI_BUFFER_SIZE 2056

// --- Variables y Objetos ---
float vReal[N_FFT];
float vImag[N_FFT];
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, N_FFT, 8000);

ESP32DMASPI::Slave slave;
uint8_t *dma_tx_buf;
uint8_t *dma_rx_buf;

void setup() {
    // Inicializar Serial solo para depuración (opcional)
    Serial.begin(115200);

    // Pin 25 es el DAC1 interno del ESP32
    pinMode(25, OUTPUT); 

    // Configura el pin como salida
    pinMode(SPEAKER_AVAIBLE, OUTPUT);
  
    // Lo inicializamos en BAJO (0 voltios) para que no haya señales falsas al arrancar
    digitalWrite(SPEAKER_AVAIBLE, HIGH);

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
    slave.begin(VSPI_HOST, VSPI_SCLK, VSPI_MISO, VSPI_MOSI, VSPI_SS);

    Serial.println("Esclavo SPI DMA VSPI Iniciado");
}

void loop() 
{
    Serial.println(digitalRead(SPEAKER_AVAIBLE));
    digitalWrite(SPEAKER_AVAIBLE, HIGH);

    // 1. Esperar y recibir la transferencia por DMA
    // Esta función bloquea hasta que el Maestro completa el envío de SPI_BUFFER_SIZE bytes
    uint32_t received_bytes = slave.transfer(dma_tx_buf, dma_rx_buf, SPI_BUFFER_SIZE);

    if (received_bytes > 0) {
        // 2. Verificar sincronía (Header y Footer)
        // El footer está en el índice 2049 (1 + 1024 + 1024)
        if (dma_rx_buf[0] == HEADER && dma_rx_buf[1 + sizeof(vReal) + sizeof(vImag)] == FOOTER) {
            
            digitalWrite(SPEAKER_AVAIBLE, LOW);

            // 3. Desempaquetar los floats del buffer DMA a los arrays de la FFT
            memcpy(vReal, &dma_rx_buf[1], sizeof(vReal));
            memcpy(vImag, &dma_rx_buf[1 + sizeof(vReal)], sizeof(vImag));

            // 4. Procesamiento: Transformada Inversa (Frecuencia -> Tiempo)
            FFT.compute(FFT_REVERSE);

            
            // 5. Reproducción por el DAC
            for (int i = 0; i < N_FFT; i++) {

                uint32_t t_inicio = micros();
                
                // Aplicamos constrain para asegurar que el valor esté en rango DAC (0-255)
                dacWrite(25, (uint8_t)constrain(vReal[i], 0, 255));
                
                // Mantener el sample rate de 8000Hz
                while ((micros() - t_inicio) < SAMPLE_PERIOD);
            }
            digitalWrite(SPEAKER_AVAIBLE, HIGH);

            
            
        } else {
            // En caso de error de sincronía, limpiamos el buffer RX
            memset(dma_rx_buf, 0, SPI_BUFFER_SIZE);
        }
    }
}
