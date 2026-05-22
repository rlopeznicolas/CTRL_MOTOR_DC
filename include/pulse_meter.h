#pragma once
#include <Arduino.h>

// Define los estados de la máquina de frecuencia
typedef enum {
    ESPERA_SUBIDA = 0,
    CONFIRMA_UNO,
    ESPERA_BAJA,
    CONFIRMA_CERO
} EstadoFrecuencia;

// Estructura para cada canal de conteo de pulsos
typedef struct {
    portMUX_TYPE mux;
    volatile EstadoFrecuencia estadoFrecuencia;
    unsigned long tiempoCambio;
    unsigned long tiempoUltimoFlanco;
    volatile unsigned long deltaFlancoMs;
    unsigned int flagFirtsMeasure;
    volatile unsigned int flagNewData;
    volatile uint32_t cntFlancos;
    int pinFrecuenciaConfig;
    unsigned long tiempoFiltroUs;
    // Añadido para caudal:
    float caudalTotal;
    float caudalInstantaneo;
    float caudalInstantaneoAcumulado;
    unsigned int nSamples;
} PulseMeterChannel;

// Instancias externas para dos canales
extern PulseMeterChannel canal1;
extern PulseMeterChannel canal2;

// Prototipos de funciones
void rstMaquinaFrecuencia(PulseMeterChannel *ch);
void iniMaquinaFrecuencia(PulseMeterChannel *ch, int pinFrecuencia, unsigned long filtroUs);
void maquinaFrecuenciaPolling(PulseMeterChannel *ch);