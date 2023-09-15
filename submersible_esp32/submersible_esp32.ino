#include <esp32/ulp.h>
#include <driver/rtc_io.h>
#include <soc/rtc.h>
#include <driver/rtc_cntl.h>

volatile int something = 0;

static void IRAM_ATTR ulp_isr(void* arg) {
    something += 1;
}

// ADC Buffer size is some integer multiple of the longest wavedescriptor.

void setup() {
    Serial.begin(115200);
    Serial.println("SERIAL OK");
    // Serial.println(rtc_fast_freq_hz);
    uint32_t values_per_cycle_1 = 9;
    uint32_t values_per_cycle_2 = 8;

    ESP_ERROR_CHECK(rtc_isr_register(&ulp_isr, NULL, RTC_CNTL_SAR_INT_ST_M));
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    
    const ulp_insn_t test[] = {
        I_MOVI(R0, 0),
        M_LABEL(1),
        // Do some measurement stuff
        I_ADDI(R0, R0, 1),
        M_BL(2, 65535),
        I_MOVI(R0, 0),
        I_WAKE(),
        M_LABEL(2),
        M_BX(1),
        I_HALT()
    };
    uint32_t size = sizeof(test) / sizeof(ulp_insn_t);
    ulp_process_macros_and_load(0, test, &size);
    // RTC_SLOW_MEM[2048 - 512] = 0;
    // Serial.println(RTC_SLOW_MEM[2048 - 512]);
    // unsigned long start = micros();
    ulp_run(0);
    // while (RTC_SLOW_MEM[2048 - 512] == 0) {
    //     delayMicroseconds(0);
    // }
    // unsigned long dt = micros() - start;
    // Serial.print("Triggered at dt=");
    // Serial.println(dt);
}

void loop() {
    // BaseObject** registry = mainObjectRegistry.getInternalRegistry();
    // uint16_t registry_capacity = mainObjectRegistry.getCurrentRegistryCapacity();
    // for (uint16_t i = 0; i < registry_capacity; i++) {
    //     if (registry[i] != NULL && registry[i]->enable_periodic) {
    //         registry[i]->periodic();
    //     }
    // }
    Serial.println(something);
    delay(500);
}