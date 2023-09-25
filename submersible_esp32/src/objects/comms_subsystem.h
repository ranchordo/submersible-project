#pragma once

#include <Arduino.h>
#include "../types/baseobject.h"
#include "../types/objects.h"
#include <driver/adc.h>

#define TRANSMIT_INTERVAL_US 1200

enum GolayCorrectionStatus {
    GOLAY_CORRECTION_OK = 0,
    GOLAY_CORRECTION_AGGR = 1,
    GOLAY_CORRECTION_NONE = 2
};

struct WaveDescriptor {
    uint32_t values_per_buffer;
    uint32_t cycles_per_buffer;
    // Wave frequency per value proportional to cpb / vpb
    int16_t* phase_1 = NULL;
    int16_t* phase_2 = NULL;
    void renderWave();
    WaveDescriptor(uint32_t vpb, uint32_t cpb) {
        this->values_per_buffer = vpb;
        this->cycles_per_buffer = cpb;
    }
    WaveDescriptor() {}
    ~WaveDescriptor() {
        if (phase_1 != NULL) {
            free(phase_1);
            phase_1 = NULL;
        }
        if (phase_2 != NULL) {
            free(phase_2);
            phase_2 = NULL;
        }
    }
};

struct ULP_ISR_Argument {
    uint16_t buffer_size;
    WaveDescriptor wave1;
    WaveDescriptor wave2;
};

struct AcousticGainParams {
    uint16_t adc_pad = 3;
    adc_atten_t adc_atten = ADC_ATTEN_DB_11;
};

enum CommunicationStatus {
    COMMS_STATUS_IDLE,
    COMMS_STATUS_INPUT,
    COMMS_STATUS_OUTPUT
};

extern volatile unsigned long dt_micros;

class CommunicationSubsystem: public BaseObject {
public:
    CommunicationSubsystem();
    CommunicationSubsystem(BaseObject** params, uint8_t num_params);
    ~CommunicationSubsystem();
    SerializationResult serialize() override {
        const char* data = "comms_subsys tx=%d rx=%d";
        int n = snprintf(NULL, 0, data, this->tx_bytes, this->rx_bytes) + 1;
        uint8_t* buffer = (uint8_t*)malloc(n);
        snprintf((char*)buffer, n, data, this->tx_bytes, this->rx_bytes);
        return { buffer, n - 1 };
    }
    uint16_t getTypeId() override { return TYPEID_COMMS_SUBSYS; }
    static uint16_t _static_typeId() { return TYPEID_COMMS_SUBSYS; }
    const char* getTypeString() override { return "Comms_Subsys"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;
    void periodic() override;
    void transmit(uint8_t* buffer, size_t len);
    void startInput();
    void startOutput();
    void startIdle();
    CommunicationStatus getStatus() { return status; }

    // private:
    uint8_t currentOffsAdj = 128;
    uint32_t tx_bytes = 0;
    uint32_t rx_bytes = 0;
    CommunicationStatus status;
    struct {
        hw_timer_t* timer;
        uint16_t timer_alloc;
    } input_periodic_timer;
    float out_amplitude = 0.7;
    // WaveDescriptor out_waves[2] = { {18, 1}, {16, 1} };
    WaveDescriptor out_waves[2] = { {9, 1}, {8, 1} };
    unsigned long rtc_fast_freq_hz;
    ULP_ISR_Argument isr_arg;
    AcousticGainParams acoustic_gain_params;
    void transmitWord(uint16_t data);
    void selectOutput(uint8_t sel);
    float getInputAmplitude(uint8_t wave);
};