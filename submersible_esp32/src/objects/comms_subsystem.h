#pragma once

#include <Arduino.h>
#include "../types/baseobject.h"
#include "../types/objects.h"
#include <driver/adc.h>

enum GolayCorrectionStatus {
    GOLAY_CORRECTION_OK = 0,
    GOLAY_CORRECTION_AGGR = 1,
    GOLAY_CORRECTION_NONE = 2
};

struct WaveDescriptor {
    uint32_t values_per_buffer;
    uint32_t cycles_per_buffer;
    // Wave frequency per value proportional to cpb / vpb
    int16_t* phase_1;
    int16_t* phase_2;
    void initWaveDescriptor(uint32_t vpb, uint32_t cpb) {
        values_per_buffer = vpb;
        cycles_per_buffer = cpb;
        phase_1 = (int16_t*)malloc(values_per_buffer * sizeof(int16_t));
        phase_2 = (int16_t*)malloc(values_per_buffer * sizeof(int16_t));
        float theta_i = 6.283185308 * cycles_per_buffer / values_per_buffer;
        for (uint32_t i = 0; i < values_per_buffer; i++) {
            float theta = theta_i * i;
            phase_1[i] = (int16_t)(sin(theta) * 32767.0);
            phase_2[i] = (int16_t)(cos(theta) * 32767.0);
        }
    }
    ~WaveDescriptor() {
        free(phase_1);
        free(phase_2);
    }
};

struct ULP_ISR_Argument {
    uint16_t buffer_size;
    WaveDescriptor wave1;
    WaveDescriptor wave2;
};

struct AcousticGainParams {
    uint16_t adc_pad = 0;
    adc_atten_t adc_atten = ADC_ATTEN_DB_0;
};

class CommunicationSubsystem: public BaseObject {
public:
    CommunicationSubsystem();
    ~CommunicationSubsystem();
    SerializationResult serialize() override {
        const char* data = "comms_subsys tx=%d rx=%d";
        int n = snprintf(NULL, 0, data, this->tx_bytes, this->rx_bytes) + 1;
        uint8_t* buffer = (uint8_t*)malloc(n);
        snprintf((char*)buffer, n, data, this->tx_bytes, this->rx_bytes);
        return {buffer, n - 1};
    }
    uint16_t getTypeId() override { return TYPEID_COMMS_SUBSYS; }
    static uint16_t _static_getTypeId() { return TYPEID_COMMS_SUBSYS; }
    const char* getTypeString() override { return "Comms_Subsys"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;
    void periodic() override;
private:
    uint32_t tx_bytes = 0;
    uint32_t rx_bytes = 0;
    unsigned long rtc_fast_freq_hz;
    ULP_ISR_Argument isr_arg = {0, {0, 0, NULL, NULL}};
    AcousticGainParams acoustic_gain_params;
    void startInput();
    void startOutput();
};