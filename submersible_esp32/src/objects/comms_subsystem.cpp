#include "comms_subsystem.h"
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>
#include <driver/rtc_cntl.h>
#include <driver/dac.h>
#include <driver/adc.h>

#define GOLAY_POLYNOMIAL 0xAE3

uint8_t computeParity(uint32_t data) {
    uint8_t parity = 0;
    for (uint8_t i = 0; i < 32; i++) {
        parity ^= (data >> i) & 1;
    }
    return parity;
}

uint8_t computeWeight(uint32_t data) {
    uint8_t weight = 0;
    for (uint8_t i = 0; i < 32; i++) {
        weight += (data >> i) & 1;
    }
    return weight;
}

uint32_t doGolay(uint32_t data) {
    data &= 0xffful;
    uint32_t data_orig = data;
    for (uint8_t i = 1; i <= 12; i++) {
        if (data & 1) {
            data ^= GOLAY_POLYNOMIAL;
        }
        data >>= 1;
    }
    uint32_t golay = (data << 12) | data_orig;
    golay |= (computeParity(golay) << 23);
    return golay;
}

uint32_t doGolaySyndrome(uint32_t data) {
    data &= 0x7ffffful;
    for (uint8_t i = 1; i <= 12; i++) {
        if (data & 1) {
            data ^= GOLAY_POLYNOMIAL;
        }
        data >>= 1;
    }
    return data << 12;
}

uint32_t ror23(uint32_t data, uint8_t i) {
    i %= 23;
    uint32_t parity = (data & (1 << 23));
    data &= ~(1 << 23);
    uint32_t mask = (1 << i) - 1;
    uint32_t rpart = (data & mask);
    uint32_t lpart = (data & (~mask));
    return (lpart >> i) | (rpart << (23 - i)) | parity;
}

uint32_t rol23(uint32_t data, uint8_t i) {
    i %= 23;
    uint32_t parity = (data & (1 << 23));
    data &= ~(1 << 23);
    uint32_t mask = (1 << (23 - i)) - 1;
    uint32_t rpart = (data & mask);
    uint32_t lpart = (data & (~mask));
    return (rpart << i) | (lpart >> (23 - i)) | parity;
}

bool tryRolledGolayCorrection(uint32_t* dataptr, uint8_t wth) {
    uint32_t data = *dataptr;
    for (uint8_t rolls = 0; rolls < 23; rolls++) {
        uint32_t syndrome = doGolaySyndrome(data);
        if (computeWeight(syndrome) <= wth) {
            data ^= syndrome;
            data = ror23(data, rolls);
            *dataptr = data;
            return true;
        }
        data = rol23(data, 1);
    }
    return false;
}

GolayCorrectionStatus tryGolayCorrection(uint32_t* dataptr) {
    if (tryRolledGolayCorrection(dataptr, 3)) {
        return GOLAY_CORRECTION_OK;
    }
    uint32_t data = *dataptr;
    for (uint8_t trial = 0; trial < 23; trial++) {
        uint32_t flip = (data ^ (1 << trial));
        if (tryRolledGolayCorrection(&flip, 2)) {
            if (!computeParity(flip)) {
                *dataptr = flip;
                return GOLAY_CORRECTION_AGGR;
            }
        }
    }
    return GOLAY_CORRECTION_NONE;
}

void interleave(uint16_t* buffer, size_t len) {
    uint16_t* result = (uint16_t*)malloc(len * 2);
    memset(result, 0, len * 2);
    for (size_t i = 0; i < 16; i++) {
        for (size_t d = 0; d < len; d++) {
            uint8_t select = (buffer[d] & (1 << i)) ? 1 : 0;
            result[(i * len + d) / 16] |= select << ((i * len + d) & 15);
        }
    }
    memcpy(buffer, result, len * 2);
    free(result);
}

void deinterleave(uint16_t* buffer, size_t len) {
    uint16_t* result = (uint16_t*)malloc(len * 2);
    memset(result, 0, len * 2);
    for (size_t i = 0; i < len; i++) {
        for (size_t b = 0; b < 16; b++) {
            size_t index = (b * len) + i;
            uint8_t select = (buffer[index / 16] & (1 << (index & 15))) ? 1 : 0;
            result[i] |= select << b;
        }
    }
    memcpy(buffer, result, len * 2);
    free(result);
}

uint8_t pad_access(uint8_t* buffer, size_t len, size_t idx) {
    if (idx >= len) {
        return 0;
    }
    return buffer[idx];
}

uint16_t* transmit(uint8_t* buffer, size_t len, size_t* txlen) {
    size_t num_golay_pairs = (len + 2) / 3;
    uint16_t* tx_buf = (uint16_t*)malloc(num_golay_pairs * 6);
    for (size_t g = 0; g < num_golay_pairs; g++) {
        size_t offs = g * 3;
        uint32_t data1 = pad_access(buffer, len, offs);
        data1 |= (pad_access(buffer, len, offs + 1) & 0x0f) << 8;
        uint32_t data2 = pad_access(buffer, len, offs + 2);
        data2 |= (pad_access(buffer, len, offs + 1) & 0xf0) << 4;
        data1 = doGolay(data1 & 0xfff);
        data2 = doGolay(data2 & 0xfff);
        tx_buf[offs + 0] = data1 & 0xffff;
        tx_buf[offs + 1] = ((data1 >> 16) & 0xff) | ((data2 & 0xff) << 8);
        tx_buf[offs + 2] = (data2 >> 8) & 0xffff;
    }
    interleave(tx_buf, num_golay_pairs * 3);
    *txlen = num_golay_pairs * 3;
    return tx_buf;
}

uint8_t* receive(uint16_t* buffer, size_t len, size_t* rxlen, GolayCorrectionStatus* status) {
    *status = GOLAY_CORRECTION_OK;
    deinterleave(buffer, len);
    size_t num_golay_pairs = len / 3;
    uint8_t* rx_buf = (uint8_t*)malloc(num_golay_pairs * 3);
    for (size_t g = 0; g < num_golay_pairs; g++) {
        size_t offs = g * 3;
        uint32_t data1 = buffer[offs + 0];
        data1 |= (buffer[offs + 1] & 0xff) << 16;
        uint32_t data2 = buffer[offs + 1] >> 8;
        data2 |= buffer[offs + 2] << 8;
        GolayCorrectionStatus corr1 = tryGolayCorrection(&data1);
        GolayCorrectionStatus corr2 = tryGolayCorrection(&data2);
        if (corr1 == GOLAY_CORRECTION_AGGR || corr2 == GOLAY_CORRECTION_AGGR) {
            *status = GOLAY_CORRECTION_AGGR;
        }
        if (corr1 == GOLAY_CORRECTION_NONE) {
            free(rx_buf);
            *status = GOLAY_CORRECTION_NONE;
            return NULL;
        }
        if (corr2 == GOLAY_CORRECTION_NONE) {
            free(rx_buf);
            *status = GOLAY_CORRECTION_NONE;
            return NULL;
        }
        rx_buf[offs] = data1 & 0xff;
        rx_buf[offs + 1] = (data1 >> 8) & 0x0f;
        rx_buf[offs + 1] |= (data2 >> 4) & 0xf0;
        rx_buf[offs + 2] = data2 & 0xff;
    }
    *rxlen = num_golay_pairs * 3;
    return rx_buf;
}

// int main() {
//     const char* datac = "Hello, I am a string, you absolute moron.";
//     uint8_t* data = (uint8_t*)malloc(strlen(datac) + 1);
//     strcpy((char*)data, datac);
//     size_t tx_len = 0;
//     uint16_t* tx_buf = transmit(data, strlen(datac) + 1, &tx_len);
//     for (int i = 0; i < tx_len; i++) {
//         for (int j = 0; j < 16; j++) {
//             uint8_t rand = (std::rand() + 3) & 0xff;
//             if (rand < 10) {
//                 tx_buf[i] ^= (1 << j);
//             }
//         }
//     }
//     free(data);
//     size_t rx_len = 0;
//     GolayCorrectionStatus st;
//     uint8_t* rx_buf = receive(tx_buf, tx_len, &rx_len, &st);
//     free(tx_buf);
//     printf("String is: %s, st is: %d\n", (char*)rx_buf, st);
//     free(rx_buf);
//     return 0;
// }

CommunicationSubsystem::CommunicationSubsystem() {
    this->enable_periodic = true;
    // Configure ULP clock
    unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
    this->rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
}

CommunicationSubsystem::~CommunicationSubsystem() {

}

BaseObject* CommunicationSubsystem::callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) {

}

void CommunicationSubsystem::periodic() {

}

inline float wave_score(WaveDescriptor wave1, uint32_t* buffer, size_t buffer_size) {
    int16_t* phase1 = wave1.phase_1;
    int16_t* phase2 = wave1.phase_2;
    uint32_t wave_size = wave1.values_per_buffer;
    uint16_t num_cycles = buffer_size / wave_size;
    float integrator_1 = 0;
    float integrator_2 = 0;

    for (uint16_t i = 0; i < num_cycles; i++) {
        for (uint16_t s = 0; s < wave_size; s++) {
            float p1 = ((float)phase1[s]) / 32767.0;
            float p2 = ((float)phase2[s]) / 32767.0;
            float val = (float)(buffer[i * wave_size + s] & 0x0fff) / 4095.0;
            integrator_1 += val * p1;
            integrator_2 += val * p2;
        }
    }

    return (integrator_1 * integrator_1) + (integrator_2 * integrator_2);
}

static void ulp_isr(void* argptr) {
    ULP_ISR_Argument* arg = (ULP_ISR_Argument*)arg;
    uint32_t* buffer = &RTC_SLOW_MEM[2048 - arg->buffer_size];
    if (buffer[0] == 0) {
        return;
    }
    float w1_score = wave_score(arg->wave1, buffer, arg->buffer_size);
    float w2_score = wave_score(arg->wave2, buffer, arg->buffer_size);
    buffer[0] = 0;
}

#define DAC_TABLE_OFFSET (2048 - 512)
#define DAC_RPT_TOGGLE (1535)
#define DAC_BUF_TOGGLE (1534)
#define DAC_RETURN_ADDR 5

void CommunicationSubsystem::startOutput() {
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 128);
    const ulp_insn_t output[] = {
        I_MOVI(R1, 0),
        M_LABEL(1),
        I_ADDR(R0, R3, R1),
        I_LD(R0, R0, 0),
        // MUST have 2*v + DAC_TABLE_OFFSET
        I_BXR(R0),
        // Return here after DAC routine
        I_ADDI(R1, R1, 1),
        I_SUBR(R0, R1, R2),
        M_BGE(2, 32768),
        I_MOVI(R1, 0),
        I_LD(R2, R1, DAC_RPT_TOGGLE),
        I_LD(R3, R1, DAC_BUF_TOGGLE),
        M_BX(1),
        M_LABEL(2),
        I_DELAY(0),
        M_BX(1),
        I_HALT()
    };
    for (int i = 0; i < 256; i++) {
        RTC_SLOW_MEM[DAC_TABLE_OFFSET + i * 2] = 0x1D4C0121 | (i << 10); //dac0
        RTC_SLOW_MEM[DAC_TABLE_OFFSET + 1 + i * 2] = 0x80000000 + DAC_RETURN_ADDR * 4;
    }
    size_t num_instr = sizeof(output) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, output, &num_instr);
    for (int i = 0; i < 16; i++) {
        uint8_t value = 128; // Silence
        uint16_t adj_value = ((uint16_t)value) * 2 + DAC_TABLE_OFFSET;
        RTC_SLOW_MEM[1024 + i] = (uint32_t)adj_value;
    }
    RTC_SLOW_MEM[DAC_RPT_TOGGLE] = 16;
    RTC_SLOW_MEM[DAC_BUF_TOGGLE] = 1024;
    ulp_run(0);
}

void CommunicationSubsystem::startInput() {
    // Setup ADC and DAC with predefined parameters
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 128);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, this->acoustic_gain_params.adc_atten);
    // Setup ULP interrupt
    uint16_t buffer_size = 1024;
    this->isr_arg.buffer_size = buffer_size;
    rtc_isr_register(&ulp_isr, (void*)(&this->isr_arg), RTC_CNTL_SAR_INT_ST_M);
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    // Write ULP loop
    const ulp_insn_t input[] = {
        I_MOVI(R0, 0),
        M_LABEL(1),
        I_ADC(R1, 0, this->acoustic_gain_params.adc_pad),
        I_ST(R1, R2, (2048 - buffer_size)),
        I_ADDI(R0, R0, 1),
        M_BL(2, buffer_size),
        I_MOVI(R0, 0),
        I_WAKE(), // Trigger ulp_isr
        M_LABEL(2),
        I_DELAY(0),
        M_BX(1),
    };
    size_t num_instr = sizeof(input) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, input, &num_instr);
    ulp_run(0);
}