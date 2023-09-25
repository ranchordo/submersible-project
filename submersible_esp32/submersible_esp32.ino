#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>
#include <driver/rtc_cntl.h>
#include "src/objects/comms_subsystem.h"

CommunicationSubsystem comms;

uint8_t* data;
size_t datalen;

void setup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    Serial.println("SERIAL OK");
    Serial.println(comms.rtc_fast_freq_hz);
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);
    comms.startInput();
    // comms.startOutput();
    // Serial.println("It is done.");
    // const char* datac = "Hello, I am a string, you absolute moron.";
    // datalen = strlen(datac) + 1;
    // data = (uint8_t*)malloc(datalen);
    // strcpy((char*)data, datac);
}

unsigned long lastPrint = 0;

void loop() {
    if (millis() - lastPrint > 125) {
        lastPrint = millis();
        Serial.print(*(float*)(&RTC_SLOW_MEM[256]));
        Serial.print(", ");
        Serial.print(comms.getInputAmplitude(1) * 1000000.0);
        Serial.print(", ");
        Serial.println(comms.getInputAmplitude(2) * 1000000.0);
    }
    // comms.periodic();
    // comms.transmit(data, datalen);
    // delay(500);
    // comms.selectOutput(2);
    // delay(1000);
    // comms.selectOutput(1);
    // delay(1000);
    // BaseObject** registry = mainObjectRegistry.getInternalRegistry();
    // uint16_t registry_capacity = mainObjectRegistry.getCurrentRegistryCapacity();
    // for (uint16_t i = 0; i < registry_capacity; i++) {
    //     if (registry[i] != NULL && registry[i]->enable_periodic) {
    //         registry[i]->periodic();
    //     }
    // }
}