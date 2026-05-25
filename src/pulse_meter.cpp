#include "pulse_meter.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

void rstMaquinaFrecuencia(PulseMeterChannel *ch) {
    portENTER_CRITICAL(&(ch->mux));
    ch->estadoFrecuencia = ESPERA_SUBIDA;
    ch->tiempoCambio = 0;
    ch->tiempoUltimoFlanco = 0;
    ch->deltaFlancoMs = 0;
    ch->flagFirtsMeasure = 0;
    ch->flagNewData = 0;
    ch->cntFlancos = 0;
    portEXIT_CRITICAL(&(ch->mux));
}

void iniMaquinaFrecuencia(PulseMeterChannel *ch, int pinFrecuencia, unsigned long filtroUs) {
    ch->pinFrecuenciaConfig = pinFrecuencia;
    ch->tiempoFiltroUs = filtroUs;
    pinMode(ch->pinFrecuenciaConfig, INPUT);
    rstMaquinaFrecuencia(ch);
}

void maquinaFrecuenciaPolling(PulseMeterChannel *ch) {
    int valorPin = digitalRead(ch->pinFrecuenciaConfig);

    switch (ch->estadoFrecuencia) {
        case ESPERA_SUBIDA:
            if (valorPin == HIGH) {
                ch->tiempoCambio = micros();
                unsigned long tiempoActualMs = millis();
                portENTER_CRITICAL(&(ch->mux));
                ch->deltaFlancoMs = tiempoActualMs - ch->tiempoUltimoFlanco;
                portEXIT_CRITICAL(&(ch->mux));
                ch->tiempoUltimoFlanco = tiempoActualMs;
                ch->estadoFrecuencia = CONFIRMA_UNO;
            }
            break;

        case CONFIRMA_UNO:
            if (valorPin == HIGH) {
                if (micros() - ch->tiempoCambio >= ch->tiempoFiltroUs) {
                    if (ch->flagFirtsMeasure == 0) {
                        ch->flagFirtsMeasure = 1;
                    } else {
                        portENTER_CRITICAL(&(ch->mux));
                        ch->flagNewData = 1;
                        ch->cntFlancos++;
                        portEXIT_CRITICAL(&(ch->mux));
                    }
                    ch->estadoFrecuencia = ESPERA_BAJA;
                }
            } else {
                ch->estadoFrecuencia = ESPERA_SUBIDA;
            }
            break;

        case ESPERA_BAJA:
            if (valorPin == LOW) {
                ch->tiempoCambio = micros();
                ch->estadoFrecuencia = CONFIRMA_CERO;
            }
            break;

        case CONFIRMA_CERO:
            if (valorPin == LOW) {
                if (micros() - ch->tiempoCambio >= ch->tiempoFiltroUs) {
                    ch->estadoFrecuencia = ESPERA_SUBIDA;
                }
            } else {
                ch->estadoFrecuencia = ESPERA_BAJA;
            }
            break;
    }
}