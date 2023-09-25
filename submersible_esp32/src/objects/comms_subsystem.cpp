#include "comms_subsystem.h"
#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>
#include <driver/rtc_cntl.h>
#include <driver/dac.h>
#include <driver/adc.h>
#include "../resource_allocator.h"

struct DualPhase {
    float p1;
    float p2;
};

DualPhase do_wave(uint32_t values_per_buffer, uint32_t cycles_per_buffer, uint32_t i) {
    i += 2; // Weird
    float theta = i * 6.283185308 * cycles_per_buffer / values_per_buffer;
    return { sin(theta), cos(theta) };
}

void WaveDescriptor::renderWave() {
    phase_1 = (int16_t*)malloc(values_per_buffer * sizeof(int16_t));
    phase_2 = (int16_t*)malloc(values_per_buffer * sizeof(int16_t));
    for (uint32_t i = 0; i < values_per_buffer; i++) {
        DualPhase data = do_wave(values_per_buffer, cycles_per_buffer, i);
        phase_1[i] = (int16_t)(data.p1 * 32767.0);
        phase_2[i] = (int16_t)(data.p2 * 32767.0);
    }
}

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

uint16_t* encode(uint8_t* buffer, size_t len, size_t* txlen) {
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

uint8_t* decode(uint16_t* buffer, size_t len, size_t* rxlen, GolayCorrectionStatus* status) {
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
    for (uint32_t i = 0; i < sizeof(out_waves) / sizeof(WaveDescriptor); i++) {
        out_waves[i].renderWave();
    }
    this->startIdle();
    this->status = COMMS_STATUS_IDLE;
}

CommunicationSubsystem::~CommunicationSubsystem() {
    for (uint32_t i = 0; i < sizeof(out_waves) / sizeof(WaveDescriptor); i++) {
        out_waves[i].~WaveDescriptor();
    }
}

BaseObject* CommunicationSubsystem::callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) {

}

void CommunicationSubsystem::periodic() {
}

#define DAC_TABLE_OFFSET (2048 - 512)
#define DAC_RPT_TOGGLE (1531)
#define DAC_BUF_TOGGLE (1530)
volatile uint32_t dac_rpt_toggle;
volatile uint32_t dac_buf_toggle;
#define DAC_RETURN_ADDR 6

void renderRTCWaveDescriptor(WaveDescriptor desc, uint32_t offset_dw, float ampl) {
    for (uint32_t i = 0; i < desc.values_per_buffer; i++) {
        float wave = do_wave(desc.values_per_buffer, desc.cycles_per_buffer, i).p2 * ampl;
        uint8_t value = (int8_t)(wave * 127.0) + 127;
        uint16_t adj_value = ((uint16_t)value) * 2 + DAC_TABLE_OFFSET;
        RTC_SLOW_MEM[offset_dw + i] = (uint32_t)adj_value;
    }
}

static IRAM_ATTR void ulp_isr_output(void* argptr) {
    RTC_SLOW_MEM[DAC_RPT_TOGGLE] = dac_rpt_toggle;
    RTC_SLOW_MEM[DAC_BUF_TOGGLE] = dac_buf_toggle;
}

void CommunicationSubsystem::startOutput() {
    this->startIdle();
    unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
    this->rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, 0);
    uint32_t base_loop_cycles = 60;
    uint32_t target_rate = 150000; // hz
    int delay_cycles = (this->rtc_fast_freq_hz / target_rate) - base_loop_cycles;
    if (delay_cycles < 0) {
        Serial.println("Output warning: can't sample at target rate.");
        delay_cycles = 0;
    }
    const ulp_insn_t output[] = {
        I_MOVI(R1, 0), // 6
        I_LD(R2, R1, DAC_RPT_TOGGLE), // 8
        I_LD(R3, R1, DAC_BUF_TOGGLE), // 8
        M_LABEL(1),
        I_ADDR(R0, R3, R1), // 6
        I_LD(R0, R0, 0), // 8
        I_BXR(R0), // 4
        // Return here after DAC routine
        I_ADDI(R1, R1, 1), // 6
        I_SUBR(R0, R1, R2), // 6
        M_BGE(2, 32768), // 4
        I_MOVI(R1, 0),
        I_LD(R2, R1, DAC_RPT_TOGGLE),
        I_LD(R3, R1, DAC_BUF_TOGGLE),
        I_WAKE(),
        M_BX(1),
        M_LABEL(2),
        I_DELAY(delay_cycles), // 6 + dt
        M_BX(1), // 4
        I_HALT()
    };
    for (int i = 0; i < 256; i++) {
        // This is the worst
        RTC_SLOW_MEM[DAC_TABLE_OFFSET + i * 2] = 0x1D4C0121 | (i << 10); // 12
        RTC_SLOW_MEM[DAC_TABLE_OFFSET + i * 2 + 1] = 0x80000000 + DAC_RETURN_ADDR * 4; // 4
    }
    size_t num_instr = sizeof(output) / sizeof(ulp_insn_t);
    rtc_isr_register(&ulp_isr_output, NULL, RTC_CNTL_SAR_INT_ST_M);
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    ulp_process_macros_and_load(0, output, &num_instr);
    RTC_SLOW_MEM[DAC_TABLE_OFFSET - 1] = DAC_TABLE_OFFSET; // Silence
    RTC_SLOW_MEM[DAC_TABLE_OFFSET - 2] = DAC_TABLE_OFFSET; // Silence
    RTC_SLOW_MEM[DAC_TABLE_OFFSET - 3] = DAC_TABLE_OFFSET; // Silence
    RTC_SLOW_MEM[DAC_TABLE_OFFSET - 4] = DAC_TABLE_OFFSET; // Silence
    for (uint8_t i = 0; i < sizeof(out_waves) / sizeof(WaveDescriptor); i++) {
        uint32_t offset = 256 + (i * 128);
        renderRTCWaveDescriptor(out_waves[i], offset, out_amplitude);
    }
    RTC_SLOW_MEM[DAC_RPT_TOGGLE] = 4;
    dac_rpt_toggle = 4;
    RTC_SLOW_MEM[DAC_BUF_TOGGLE] = DAC_TABLE_OFFSET - 4;
    dac_buf_toggle = DAC_TABLE_OFFSET - 4;
    ulp_run(0);
    this->status = COMMS_STATUS_OUTPUT;
}

void CommunicationSubsystem::selectOutput(uint8_t sel) {
    if (sel == 0) {
        dac_rpt_toggle = 4;
        dac_buf_toggle = DAC_TABLE_OFFSET - 4;
        return;
    }
    dac_rpt_toggle = out_waves[sel - 1].values_per_buffer - 1;
    dac_buf_toggle = 256 + (128 * (sel - 1));
}

float wave_score(WaveDescriptor* wave, void* argptr) {
    ULP_ISR_Argument* arg = (ULP_ISR_Argument*)argptr;
    int16_t* phase1 = wave->phase_1;
    int16_t* phase2 = wave->phase_2;
    uint32_t wave_size = wave->values_per_buffer;
    uint16_t num_cycles = arg->buffer_size / wave_size;
    float integrator_1 = 0;
    float integrator_2 = 0;
    for (uint16_t i = 0; i < num_cycles * wave_size; i++) {
        float p1 = ((float)phase1[i % wave_size]) / 32767.0;
        float p2 = ((float)phase2[i % wave_size]) / 32767.0;
        float val = (RTC_SLOW_MEM[2048 - (2 * arg->buffer_size) + i] & 0x0fff) / 4095.0;
        integrator_1 += val * p1;
        integrator_2 += val * p2;
    }
    integrator_1 = integrator_1 / (num_cycles * wave_size);
    integrator_1 = integrator_1 * integrator_1;
    integrator_2 = integrator_2 / (num_cycles * wave_size);
    integrator_2 = integrator_2 * integrator_2;
    return integrator_1 + integrator_2;
}

uint32_t to_uint32(float data) {
    return *(uint32_t*)(&data);
}

uint32_t cp0_regs[18];

volatile unsigned long dt_micros;
volatile unsigned long t_micros = 0;

static IRAM_ATTR void ulp_isr_input(void* argptr) {
    ULP_ISR_Argument* arg = (ULP_ISR_Argument*)argptr;
    dt_micros = micros() - t_micros;
    t_micros = micros();
    if (RTC_SLOW_MEM[2048 - arg->buffer_size] == 0) {
        return;
    }
    for (uint32_t i = 0; i < arg->buffer_size; i++) {
        RTC_SLOW_MEM[2048 - (2 * arg->buffer_size) + i] =
            RTC_SLOW_MEM[2048 - arg->buffer_size + i];
    }
    uint32_t cpstate = xthal_get_cpenable();
    if (!cpstate) {
        xthal_set_cpenable(1);
    }
    xthal_save_cp0(cp0_regs);
    float adc_avg = 0;
    for (uint32_t i = 0; i < arg->buffer_size; i++) {
        adc_avg += RTC_SLOW_MEM[2048 - (2 * arg->buffer_size) + i] & 0x0fff;
    }
    adc_avg /= 4095.0 * arg->buffer_size;
    RTC_SLOW_MEM[256] = to_uint32(adc_avg);
    RTC_SLOW_MEM[257] = to_uint32(wave_score(&arg->wave1, argptr));
    RTC_SLOW_MEM[258] = to_uint32(wave_score(&arg->wave2, argptr));
    RTC_SLOW_MEM[2048 - arg->buffer_size] = 0;
    xthal_restore_cp0(cp0_regs);
    if (!cpstate) {
        xthal_set_cpenable(0);
    }
}

static CommunicationSubsystem* input_timer_isr_ctx;

static IRAM_ATTR void input_timer_isr() {
    float adc_avg = input_timer_isr_ctx->getInputAmplitude(0);
    if (adc_avg > 0.7) {
        input_timer_isr_ctx->currentOffsAdj++;
        dac_output_voltage(DAC_CHANNEL_1, input_timer_isr_ctx->currentOffsAdj);
    }
    if (adc_avg < 0.5) {
        input_timer_isr_ctx->currentOffsAdj--;
        dac_output_voltage(DAC_CHANNEL_1, input_timer_isr_ctx->currentOffsAdj);
    }
}

void CommunicationSubsystem::startInput() {
    this->startIdle();
    unsigned long rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 1000);
    this->rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period;
    // Setup ADC and DAC with predefined parameters
    this->isr_arg.buffer_size = 256;
    this->isr_arg.wave1 = this->out_waves[0];
    this->isr_arg.wave2 = this->out_waves[1];
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_voltage(DAC_CHANNEL_1, currentOffsAdj);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, this->acoustic_gain_params.adc_atten);
    // Setup ULP interrupt
    rtc_isr_register(&ulp_isr_input, (void*)(&this->isr_arg), RTC_CNTL_SAR_INT_ST_M);
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    uint32_t base_loop_cycles = 90;
    uint32_t target_rate = 75000;
    int delay_cycles = (this->rtc_fast_freq_hz / target_rate) - base_loop_cycles;
    // Write ULP loop
    const ulp_insn_t input[] = {
        I_MOVI(R0, 0),
        M_LABEL(1),
        I_ADC(R1, 0, this->acoustic_gain_params.adc_pad), // ?
        I_ST(R1, R0, (2048 - isr_arg.buffer_size)), // 8
        I_ADDI(R0, R0, 1), // 6
        M_BL(2, isr_arg.buffer_size), // 4
        I_MOVI(R0, 0),
        I_WAKE(), // Trigger ulp_isr
        M_BX(1),
        M_LABEL(2),
        I_DELAY(delay_cycles), // 6 + dt
        M_BX(1), // 4
    };
    size_t num_instr = sizeof(input) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, input, &num_instr);
    ulp_run(0);
    bool timer_int_alloc_success;
    uint16_t timer_id = ResourceAllocatorPresets::hw_timer_ints.allocate(&timer_int_alloc_success);
    if (timer_int_alloc_success) {
        input_timer_isr_ctx = this;
        input_periodic_timer.timer_alloc = timer_id;
        input_periodic_timer.timer = timerBegin(timer_id, 80, true);
        timerAttachInterrupt(input_periodic_timer.timer, &input_timer_isr, true);
        timerAlarmWrite(input_periodic_timer.timer, 10000, true);
        timerAlarmEnable(input_periodic_timer.timer);
    }
    this->status = COMMS_STATUS_INPUT;
}

float CommunicationSubsystem::getInputAmplitude(uint8_t wave) {
    return *(float*)(&RTC_SLOW_MEM[256 + wave]);
}

void CommunicationSubsystem::transmitWord(uint16_t data) {
    for (uint8_t i = 0; i < 16; i++) {
        this->selectOutput((data & 1) + 1);
        delayMicroseconds(TRANSMIT_INTERVAL_US);
        data >>= 1;
    }
    this->selectOutput(0);
    delayMicroseconds(TRANSMIT_INTERVAL_US);
}

// Does not free the buffer.
void CommunicationSubsystem::transmit(uint8_t* buffer, size_t len) {
    size_t tx_len = 0;
    uint16_t* tx_buf = encode(buffer, len, &tx_len);
    for (size_t i = 0; i < tx_len; i++) {
        this->transmitWord(tx_buf[i]);
    }
    free(tx_buf);
}

void CommunicationSubsystem::startIdle() {
    if (this->status == COMMS_STATUS_INPUT) {
        timerAlarmDisable(input_periodic_timer.timer);
        timerDetachInterrupt(input_periodic_timer.timer);
        timerEnd(input_periodic_timer.timer);
        this->input_periodic_timer.timer = NULL;
        ResourceAllocatorPresets::hw_timer_ints.freeAllocation(
            input_periodic_timer.timer_alloc);
    }
    const ulp_insn_t halt_code[] = {
        I_HALT(),
        I_END(),
    };
    size_t num_instrs = sizeof(halt_code) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, halt_code, &num_instrs);
    ulp_run(0); // Shutdown ULP coproc
    this->status = COMMS_STATUS_IDLE;
}