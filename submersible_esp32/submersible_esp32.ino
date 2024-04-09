#include "src/registry.h"
#include "src/objects/comms_subsystem.h"
#include "src/startup.h"

#include <esp32/ulp.h>

#define MAX_UNTENURED_AGE 2000

CommunicationSubsystem comms;

void setup() {
    // Setup critical live objects
    Serial.begin(115200);
    Serial.println("SERIAL OK");
    comms.startInput();
    for (int i = 0; i < 256; i++) {
        RTC_SLOW_MEM[2048 - (2 * 256) + i] = 0;
    }
    delay(200);
    // pinMode(4, OUTPUT);
    // digitalWrite(4, LOW);
    // comms.startOutput();

    // runStartupProcedure();
}

// uint8_t packet1[17] = { 'H', 'e', 'l', 'l', 'o', ',', ' ', 'I', ' ', 'a', 'm', ' ', 'a', ' ', 's', 't', 1 };
// uint8_t packet2[17] = { 'r', 'i', 'n', 'g', ',', ' ', 'y', 'o', 'u', ' ', 'a', 'b', 's', 'o', 'l', 'u', 1 };
// uint8_t packet3[17] = { 't', 'e', ' ', 'm', 'o', 'r', 'o', 'n', '.',  0 ,  0 ,  0 ,  0 ,  0 ,  0 ,  0 , 0 };

uint32_t parsepoint = 0;
double intervals_per_us = 0.000292969;
struct {
    unsigned long intervals = 0;
    unsigned long start_time = 0;
    uint32_t prev_waveidx = 0;
} interval_rate_calc;
uint8_t parser_state = 0;

uint8_t parser_consume_waves(uint32_t* idx, uint32_t num_waves) {
    uint32_t total = 0;
    for (uint32_t i = 0; i < num_waves; i++) {
        total += waves[*idx];
        *idx = *idx + 1;
        *idx = (*idx) % COMMS_WAVEBUF_SIZE;
    }
    // Serial.print("Total ");
    // Serial.print(total);
    // Serial.print(" num_waves ");
    // Serial.println(num_waves);
    if (total > (num_waves / 2)) {
        return 1;
    }
    return 0;
}

void loop() {
    // TODO: When transferring reset interval_rate_calc
    uint32_t grabbed_waveidx = waveidx;
    const uint32_t rem_waves = ((int32_t)grabbed_waveidx - (int32_t)parsepoint + COMMS_WAVEBUF_SIZE) % COMMS_WAVEBUF_SIZE;
    interval_rate_calc.intervals +=
        ((int32_t)grabbed_waveidx - (int32_t)interval_rate_calc.prev_waveidx + COMMS_WAVEBUF_SIZE) % COMMS_WAVEBUF_SIZE;
    interval_rate_calc.prev_waveidx = grabbed_waveidx;
    if (micros() - interval_rate_calc.start_time > 1000000) {
        double interval_rate = interval_rate_calc.intervals / (double)(micros() - interval_rate_calc.start_time);
        intervals_per_us = interval_rate;
        interval_rate_calc.start_time = micros();
        interval_rate_calc.intervals = 0;
    }
    // Serial.print(rem_waves);
    // Serial.print(", ");
    // Serial.println(parsepoint);
    // delay(100);
    if (parser_state == 0) {
        // Waiting for a window...
        const uint32_t idle_target = (double)(20 * TRANSMIT_INTERVAL_US) * intervals_per_us;
        if (rem_waves >= idle_target) {
            uint32_t idx = parsepoint;
            uint8_t last_wave = 0;
            bool panic_mode = false;
            for (uint32_t counter = 0; counter < idle_target; counter++) {
                if (!panic_mode) {
                    if (waves[idx] == 1 && last_wave == 1) {
                        // Nope
                        panic_mode = true;
                    }
                } else {
                    if (waves[idx] == 0) {
                        break;
                    }
                }
                last_wave = waves[idx];
                idx++;
                idx %= COMMS_WAVEBUF_SIZE;
            }
            parsepoint = idx;
            if (!panic_mode) {
                parser_state = 1;
            }
        }
    } else if (parser_state == 1) {
        // Parse until we hit non-idle
        if (rem_waves > (double)(8 * TRANSMIT_INTERVAL_US) * intervals_per_us) {
            // Over 8 transmit intervals of silence, wait for inactivity again
            parser_state = 0;
        }
        uint32_t idx = parsepoint;
        uint8_t last_wave = 0;
        for (uint32_t counter = 0; counter < rem_waves; counter++) {
            if (waves[idx] == 1 && last_wave == 1) {
                parsepoint = idx - 1;
                parser_state = 2;
                break;
            }
            last_wave = waves[idx];
            idx++;
            idx %= COMMS_WAVEBUF_SIZE;
        }
    } else if (parser_state == 2) {
        // Parse and emit a word
        const uint32_t word_target = (double)(20 * TRANSMIT_INTERVAL_US) * intervals_per_us;
        if (rem_waves >= word_target) {
            Serial.println("Parser state 2!");
            uint32_t idx = parsepoint;
            uint32_t target_offs = (double)(1 * TRANSMIT_INTERVAL_US) * intervals_per_us;
            uint8_t startbit = parser_consume_waves(&idx, target_offs + parsepoint - idx);
            if (startbit == 0) {
                // Sadge
                parser_state = 0;
                Serial.println("Start bit missing");
                return;
            }
            uint16_t word = 0;
            for (uint32_t i = 0; i < 16; i++) {
                target_offs = (double)((i + 2) * TRANSMIT_INTERVAL_US) * intervals_per_us;
                uint8_t bit = parser_consume_waves(&idx, target_offs + parsepoint - idx);
                word |= ((uint16_t)bit) << (16 - i - 1);
            }
            target_offs = (double)(18 * TRANSMIT_INTERVAL_US) * intervals_per_us;
            uint8_t stopbit = parser_consume_waves(&idx, target_offs + parsepoint - idx);
            if (stopbit == 1) {
                // Sadge
                parser_state = 0;
                Serial.println("Stop bit missing");
                return;
            }
            Serial.println(word);
            parsepoint = idx;
            parser_state = 1;
        }
    }

    // for (int i = 0; i < 18; i++) {
    //     comms.transmitWord(27624 + i);
    // }
    // delayMicroseconds(20 * TRANSMIT_INTERVAL_US);

    // comms.transmit(packet1, 17);
    // comms.transmit(packet2, 17);
    // comms.transmit(packet3, 17);
    // comms.startIdle();
    // comms.startOutput();
    // delay(500);

    // Periodic
    // mainObjectRegistry.runAllLivePeriodic();
    // mainObjectRegistry.runGarbageCollection(MAX_UNTENURED_AGE);
}