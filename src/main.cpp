#include <Arduino.h>
#include <M5Unified.h>
#include "pulse_meter.h"

// Pines de Hardware
const int PIN_DRV_IN1 = 26;
const int PIN_DRV_IN2 = 0;
const int PIN_HALL    = 25;
const int BTN_UP      = 32;
const int BTN_DOWN    = 33;

// PWM ESP32
const int PWM_CHANNEL = 0;
const int PWM_FREQ    = 20000;
const int PWM_RES     = 10;
const int PWM_MAX     = 1023;

// Ajustes de control
const float MAX_RPM_SETPOINT = 1200.0f;

// PWM de arranque y mínimo útil
const int PWM_START = 100;      // Ajustar en banco
const int PWM_MIN   = 75;      // Ajustar según zona muerta del motor
const unsigned long START_TIME_MS = 250;

// Variables de medición
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulseInterval = 0;
float rpmActual = 0;
float rpmInstant = 0;
float rpmTarget = 0;
float rpmDisplay = 0;
unsigned long lastInterval = 0;

// Variables PID
unsigned long lastUpdate = 0;
float Kp = 0.80f;
float Ki = 1.00f;
float Kd = 0.00;
float integral = 0, lastError = 0;

// Estado de arranque
bool motorRunning = false;
unsigned long startBoostStart = 0;

PulseMeterChannel canal1 = {portMUX_INITIALIZER_UNLOCKED, ESPERA_SUBIDA, 0, 0, 0, 0, 0, 0, PIN_HALL, 1000, 0, 0, 0, 0};

// Tarea que se ejecuta en core 0
void tareaFrecuencia(void *pvParameters);

void setup() {
    pinMode(PIN_DRV_IN1, OUTPUT);
    pinMode(PIN_DRV_IN2, OUTPUT);
    digitalWrite(PIN_DRV_IN1, LOW);
    digitalWrite(PIN_DRV_IN2, LOW);

    auto cfg = M5.config();
    M5.begin(cfg);
    Serial.begin(115200);

    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_HALL, INPUT_PULLUP);

    ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);
    ledcAttachPin(PIN_DRV_IN1, PWM_CHANNEL);
    ledcWrite(PWM_CHANNEL, 0);

    iniMaquinaFrecuencia(&canal1, PIN_HALL, 100); // Inicializar el canal 1
    xTaskCreatePinnedToCore(
      tareaFrecuencia,      // Función de la tarea
      "TareaFrecuencia",    // Nombre de la tarea
      2048,                 // Stack size
      NULL,                 // Parámetro
      1,                    // Prioridad
      NULL,                 // Handle
      0                     // Core 0
    );

    M5.Display.setRotation(3);
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(2);
}

void loop() {
    M5.update();

    static unsigned long lastBtnAction = 0;
    static unsigned long btnPressStart = 0;
    static bool btnActive = false;

    if (digitalRead(BTN_UP) == LOW) {
        if (!btnActive) {
            if (millis() - lastBtnAction > 200) {
                rpmTarget = min(rpmTarget + 10.0f, MAX_RPM_SETPOINT);
                lastBtnAction = millis();
                btnPressStart = millis();
                btnActive = true;
            }
        } else if (millis() - btnPressStart > 500) {
            if (millis() - lastBtnAction > 50) {
                rpmTarget = min(rpmTarget + 20.0f, MAX_RPM_SETPOINT);
                lastBtnAction = millis();
            }
        }
    } else if (digitalRead(BTN_DOWN) == LOW) {
        if (!btnActive) {
            if (millis() - lastBtnAction > 200) {
                rpmTarget = max(rpmTarget - 10.0f, 0.0f);
                lastBtnAction = millis();
                btnPressStart = millis();
                btnActive = true;
            }
        } else if (millis() - btnPressStart > 500) {
            if (millis() - lastBtnAction > 50) {
                rpmTarget = max(rpmTarget - 20.0f, 0.0f);
                lastBtnAction = millis();
            }
        }
    } else {
        btnActive = false;
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= 100) {
        lastUpdate = currentMillis;

        unsigned long interval = 0;
        unsigned long lastPulse = lastPulseMicros;
        unsigned long timeSinceLastPulse = micros() - lastPulseMicros;
        // --- CANAL 1 ---
        portENTER_CRITICAL(&(canal1.mux));
        if (canal1.flagNewData) {
            canal1.flagNewData = 0;

            if (canal1.deltaFlancoMs != 0) {
            interval = canal1.deltaFlancoMs;
            lastPulseMicros = micros();
            }
        }
        portEXIT_CRITICAL(&(canal1.mux));

        if (timeSinceLastPulse > 1000000) {
            rpmActual = 0;
            rpmInstant = 0;
        } else if (interval > 0) {
            rpmInstant = 60000000.0f / interval;
        }
        rpmActual= rpmActual * 0.5f + rpmInstant * 0.5f; // Filtro simple para suavizar lectura

        Serial.print(interval);
        Serial.print(", ");
        Serial.println(lastInterval);

        int pwmOutput = 0;

        if (rpmTarget > 0) {
            if (!motorRunning) {
                motorRunning = true;
                startBoostStart = millis();
                integral = 0;
                lastError = 0;
            }

            if ((millis() - startBoostStart) < START_TIME_MS && rpmActual < 100.0f) {
                pwmOutput = PWM_START;
            } else {
                float error = rpmTarget - rpmActual;
                integral = constrain(integral + error * 0.1f, -400.0f, 400.0f);
                float derivative = (error - lastError) / 0.1f;
                float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

                pwmOutput = constrain((int)output, 0, PWM_MAX);

                if (pwmOutput > 0 && pwmOutput < PWM_MIN) {
                    pwmOutput = PWM_MIN;
                }

                lastError = error;
            }
        } else {
            pwmOutput = 0;
            integral = 0;
            lastError = 0;
            motorRunning = false;
        }

        ledcWrite(PWM_CHANNEL, pwmOutput);

        M5.Display.setTextColor(WHITE, BLACK);
        M5.Display.setCursor(10, 15);
        M5.Display.printf("TARGET: %.0f   ", rpmTarget);

        rpmDisplay = rpmActual*0.2f + rpmDisplay*0.8f; // Mostrar valor suavizado
        M5.Display.setCursor(10, 45);
        M5.Display.setTextColor(YELLOW, BLACK);
        M5.Display.printf("REAL: %.0f    ", rpmDisplay);

        M5.Display.setCursor(10, 75);
        M5.Display.setTextColor(CYAN, BLACK);
        float pwmPercent = (pwmOutput / (float)PWM_MAX) * 100.0f;
        M5.Display.printf("PWM: %.1f%%   ", pwmPercent);
    }
}

void tareaFrecuencia(void *pvParameters) {
  while (1) {
    maquinaFrecuenciaPolling(&canal1); // Polling del canal 1
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}