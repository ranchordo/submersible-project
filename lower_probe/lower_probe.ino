#include <Arduino.h>
#include <driver/spi_slave.h>
#include <QMC5883LCompass.h>

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
#error Unsupported esp32-arduino version, update ledc stuff
#endif

#define ESC_CONTROL_PIN 26
#define DAC_OUTPUT_PIN 25
#define LIGHT_CONTROL_PIN 32
#define PITCH_UP_PIN 13
#define PITCH_DOWN_PIN 27

#define LIGHT_LEDC_CHANNEL 0
#define ESC_LEDC_CHANNEL 1
#define PITCH_LEDC_CHANNEL 2

#define QMC5883L_ADDRESS 0x0C

QMC5883LCompass qmc5883l;

bool pitch_ledc_attached_up = true;

struct spi_tx {
    uint16_t magic = 0xb0a7;
    uint16_t heading;
    uint16_t pitch;
    uint8_t lighting;
    uint8_t dac;
};

void write_pitch_signal(int value) {
    if (value >= 0) {
        if (!pitch_ledc_attached_up) {
            ledcDetachPin(PITCH_DOWN_PIN);
            ledcAttachPin(PITCH_UP_PIN, PITCH_LEDC_CHANNEL);
        }
        ledcWrite(PITCH_LEDC_CHANNEL, (uint32_t)(+value));
    } else {
        if (pitch_ledc_attached_up) {
            ledcDetachPin(PITCH_UP_PIN);
            ledcAttachPin(PITCH_DOWN_PIN, PITCH_LEDC_CHANNEL);
        }
        ledcWrite(PITCH_LEDC_CHANNEL, (uint32_t)(-value));
    }
}

void setup() {
    dacWrite(DAC_OUTPUT_PIN, 0);
    ledcSetup(LIGHT_LEDC_CHANNEL, 20000, 10);
    ledcSetup(ESC_LEDC_CHANNEL, 50, 10);
    ledcSetup(PITCH_LEDC_CHANNEL, 100, 10);
    ledcAttachPin(LIGHT_CONTROL_PIN, LIGHT_LEDC_CHANNEL);
    ledcAttachPin(ESC_CONTROL_PIN, ESC_LEDC_CHANNEL);
    ledcAttachPin(PITCH_UP_PIN, PITCH_LEDC_CHANNEL);
    pitch_ledc_attached_up = true;
    ledcWrite(LIGHT_LEDC_CHANNEL, 0);
    ledcWrite(ESC_LEDC_CHANNEL, 28);
    ledcWrite(PITCH_LEDC_CHANNEL, 0);

    Serial.begin(115200);
    Serial.println("SERIAL OK");
    qmc5883l.setADDR(QMC5883L_ADDRESS);
    qmc5883l.init();
    qmc5883l.clearCalibration();
}

void loop() {
    qmc5883l.read();
    Serial.print("x:");
    Serial.print(qmc5883l.getX());
    Serial.print(",y:");
    Serial.print(qmc5883l.getY());
    Serial.print(",z:");
    Serial.println(qmc5883l.getZ());
    delay(100);
}