#include <Arduino.h>
#include <M5Unified.h>

// Pines de Hardware (Mantenidos según tu diseño)
const int PIN_DRV_IN1 = 26; 
const int PIN_DRV_IN2 = 0;  
const int PIN_HALL    = 25;
const int BTN_UP      = 32;
const int BTN_DOWN    = 33;

// Variables de Medición (Sensor Hall)
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulseInterval = 0;
float rpmActual = 0;
float rpmTarget = 0;
const float MAX_RPM = 1200.0;

// Variables de Control (PID)
unsigned long lastUpdate = 0;
float Kp = 0.6, Ki = 0.12, Kd = 0.03; 
float integral = 0, lastError = 0;

// Interrupción: Medir tiempo entre flancos de subida
void IRAM_ATTR handlePulse() {
    unsigned long now = micros();
    unsigned long diff = now - lastPulseMicros;
    if (diff > 5000) { // Filtro de ruido 5ms
        pulseInterval = diff;
        lastPulseMicros = now;
    }
}

void setup() {
    // 1. Seguridad de Hardware
    pinMode(PIN_DRV_IN1, OUTPUT);
    pinMode(PIN_DRV_IN2, OUTPUT);
    digitalWrite(PIN_DRV_IN1, LOW);
    digitalWrite(PIN_DRV_IN2, LOW);

    // 2. Inicialización M5Unified
    auto cfg = M5.config();
    M5.begin(cfg);

    // 3. Configuración de Botones Externos y Sensor
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_HALL, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_HALL), handlePulse, RISING);

    // 4. Configuración PWM (ESP32)
    ledcSetup(0, 20000, 10); // Canal 0, 20kHz, 10 bits
    ledcAttachPin(PIN_DRV_IN1, 0);
    ledcWrite(0, 0);

    // 5. Configuración de Pantalla (M5Unified usa M5.Display)
    M5.Display.setRotation(1);
    M5.Display.fillScreen(BLACK);
    M5.Display.setTextSize(2); // Tamaño recuperado
}

void loop() {
    M5.update(); 

    // Bloque de Botones Externos (G32 y G33)
    if (digitalRead(BTN_UP) == LOW) {
        rpmTarget = min(rpmTarget + 100.0f, MAX_RPM);
        delay(150); 
    }
    if (digitalRead(BTN_DOWN) == LOW) {
        rpmTarget = max(rpmTarget - 100.0f, 0.0f);
        delay(150);
    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastUpdate >= 100) {
        lastUpdate = currentMillis;

        // Cálculo de RPM con timeout de 1s
        noInterrupts();
        unsigned long interval = pulseInterval;
        unsigned long timeSinceLastPulse = micros() - lastPulseMicros;
        interrupts();

        if (timeSinceLastPulse > 1000000) {
            rpmActual = 0;
        } else if (interval > 0) {
            rpmActual = 60000000.0 / interval;
        }

        // Algoritmo PID
        int pwmOutput = 0;
        if (rpmTarget > 0) {
            float error = rpmTarget - rpmActual;
            integral = constrain(integral + error * 0.1, -500, 500);
            float derivative = (error - lastError) / 0.1;
            float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
            pwmOutput = constrain((int)output, 0, 1023);
            lastError = error;
        } else {
            pwmOutput = 0;
            integral = 0;
            lastError = 0;
        }

        ledcWrite(0, pwmOutput);

        // Visualización con M5.Display (Compatible con M5Unified)
        M5.Display.setTextColor(WHITE, BLACK);
        M5.Display.setCursor(10, 15);
        M5.Display.printf("TARGET:%.0f  ", rpmTarget);
        
        M5.Display.setCursor(10, 45);
        M5.Display.setTextColor(YELLOW, BLACK);
        M5.Display.printf("REAL:%.1f   ", rpmActual);

        M5.Display.setCursor(10, 75);
        M5.Display.setTextColor(CYAN, BLACK);
        float pwmPercent = (pwmOutput / 1023.0) * 100.0;
        M5.Display.printf("PWM: %.1f%%  ", pwmPercent);
    }
}
