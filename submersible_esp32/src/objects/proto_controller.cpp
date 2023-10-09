#include "proto_controller.h"
#include "../resource_allocator.h"
#include "../registry.h"
#include "../types/primitives.h"
#include "../types/objects.h"
#include <setjmp.h>

void* proto_timer_isr_ctx = NULL;

#define LONG_RESPONSE(arg, dptr, len) \
{ \
    xSemaphoreTake(arg->buffer_semaphore, portMAX_DELAY); \
    for (uint8_t i = 0; i < len; i++) { \
        arg->tx_buffer[arg->tx_buffer_top] = dptr[i]; \
        arg->tx_buffer_top++; \
    } \
    arg->force_packet = true; \
    xSemaphoreGive(arg->buffer_semaphore); \
}

#define SHORT_RESPONSE(arg, data...) \
{ \
    uint8_t d[] = {data}; \
    LONG_RESPONSE(arg, d, sizeof(d)); \
}

#define CHECK_OBJ(arg, object) if (object == NULL) { SHORT_RESPONSE(arg, 0xe2); break; }

struct InStreamParser {
    InStreamParser(uint8_t* instream, uint32_t available, int* at_end) {
        this->instream = instream;
        this->available = available;
        this->at_end = at_end;
    }
    void advance() {
        inptr++;
        if (inptr % 17 == 16) {
            inptr++;
        }
        if (inptr >= available) {
            longjmp(this->at_end, 1);
            inptr = available - 1;
        }
    }
    uint8_t peekByte() {
        return instream[inptr];
    }
    uint8_t parseByte() {
        uint8_t ret = peekByte();
        advance();
        return ret;
    }
    uint16_t parseWord() {
        uint8_t low = peekByte();
        advance();
        uint8_t high = peekByte();
        advance();
        return low | (high << 8);
    }
    uint8_t* remaining() {
        return instream + inptr;
    }
    uint32_t inptr = 0;
    int* at_end;
    uint32_t available;
    uint8_t* instream;
};

IRAM_ATTR void proto_timer_isr() {
    ProtocolController* arg = (ProtocolController*)proto_timer_isr_ctx;
    uint32_t available = arg->comms->available();
    if (available < 17) {
        return;
    }
    uint32_t last_packet_start = available - (available % 17) - 17;
    if (arg->comms->getInstream()[last_packet_start + 16] == 0) {
        jmp_buf proto_error;
        int ptoerr = setjmp(proto_error);
        if (ptoerr > 0) {
            SHORT_RESPONSE(arg, 0xe + ptoerr);
        } else {
            // Parse through the instream
            InStreamParser parser(arg->comms->getInstream(), available, proto_error);
            switch (parser.parseByte()) {
            case 0xfa:
                // System-level commands
                switch (parser.parseByte()) {
                case 0xc0:
                    for (;;) { ESP.restart(); }
                    break;
                case 0xc1:
                    SHORT_RESPONSE(arg, 0x5a);
                    break;
                default:
                    SHORT_RESPONSE(arg, 0xe1);
                }
                break;
            case 0xfb:
                // Resource-level commands
                switch (parser.parseByte()) {
                case 0x01: {
                    uint16_t typeId = parser.parseWord();
                    uint8_t datalen = parser.parseByte();
                    BaseObject* prim = constructPrimitive(typeId, { parser.remaining(), datalen });
                    uint16_t handle = mainObjectRegistry.addObject(prim);
                    SHORT_RESPONSE(arg, 0x5b, handle & 0xff, handle >> 8);
                    break;
                }
                case 0x02: {
                    uint16_t typeId = parser.parseWord();
                    uint8_t handlelen = parser.parseByte();
                    BaseObject** params = (BaseObject**)malloc(handlelen * sizeof(void*));
                    for (uint8_t i = 0; i < handlelen; i++) {
                        uint16_t handle = parser.parseWord();
                        params[i] = mainObjectRegistry.getObject(handle);
                    }
                    BaseObject* obj = constructObject(typeId, params, handlelen);
                    free(params);
                    uint16_t handle = mainObjectRegistry.addObject(obj);
                    SHORT_RESPONSE(arg, 0x5b, handle & 0xff, handle >> 8);
                    break;
                }
                case 0x03: {
                    uint16_t handle = parser.parseWord();
                    BaseObject* obj = mainObjectRegistry.getObject(handle);
                    CHECK_OBJ(arg, obj);
                    uint8_t slot = parser.parseByte();
                    uint8_t handlelen = parser.parseByte();
                    BaseObject** params = (BaseObject**)malloc(handlelen * sizeof(void*));
                    for (uint8_t i = 0; i < handlelen; i++) {
                        uint16_t handle = parser.parseWord();
                        params[i] = mainObjectRegistry.getObject(handle);
                    }
                    BaseObject* ret = obj->callMethod(slot, params, handlelen);
                    free(params);
                    if (ret == NULL) {
                        SHORT_RESPONSE(arg, 0x5a);
                        break;
                    }
                    uint16_t ret_handle = mainObjectRegistry.addObject(ret);
                    if (ret->isException()) {
                        SHORT_RESPONSE(arg, 0x5e, ret_handle & 0xff, ret_handle >> 8);
                        break;
                    }
                    if (ret->isPreviewable()) {
                        SHORT_RESPONSE(arg, 0x5c, ret->preview(), ret_handle & 0xff, ret_handle >> 8);
                        break;
                    }
                    SHORT_RESPONSE(arg, 0x5b, ret_handle & 0xff, ret_handle >> 8);
                    break;
                }
                case 0x04: {
                    uint16_t handle = parser.parseWord();
                    BaseObject* obj = mainObjectRegistry.getObject(handle);
                    CHECK_OBJ(arg, obj);
                    SerializationResult ser = obj->serialize();
                    // Compound response syntax (weird)
                    arg->force_packet = false; // Hold short response
                    xSemaphoreTake(arg->buffer_semaphore, portMAX_DELAY);
                    arg->tx_buffer[arg->tx_buffer_top] = 0x5d;
                    arg->tx_buffer_top++;
                    xSemaphoreGive(arg->buffer_semaphore);
                    LONG_RESPONSE(arg, ser.buffer, ser.datalen);
                    // Finally
                    free(ser.buffer);
                    break;
                }
                case 0x05: {
                    BaseObject* obj = mainObjectRegistry.getObject(parser.parseWord());
                    CHECK_OBJ(arg, obj);
                    const char* typestr = obj->getTypeString();
                    PString* pstr = new PString(typestr, false);
                    uint16_t handle = mainObjectRegistry.addObject(pstr);
                    SHORT_RESPONSE(arg, 0x5b, handle & 0xff, handle >> 8);
                    break;
                }
                case 0x06: {
                    uint16_t handle = parser.parseWord();
                    if (!mainObjectRegistry.exists(handle)) {
                        CHECK_OBJ(arg, NULL);
                    }
                    mainObjectRegistry.delObject(handle);
                    break;
                }
                case 0x07: {
                    BaseObject* obj = mainObjectRegistry.getObject(parser.parseWord());
                    CHECK_OBJ(arg, obj);
                    obj->tenure();
                    break;
                }
                default:
                    SHORT_RESPONSE(arg, 0xe1);
                }
                break;
            default:
                SHORT_RESPONSE(arg, 0xe1);
            }
        }
        arg->comms->consumeInputBytes(last_packet_start + 17);
    }
}

void IRAM_ATTR proto_timer_isr_module() {
    ProtocolController* arg = (ProtocolController*)proto_timer_isr_ctx;
    uint32_t available = arg->comms->available();
    if (available > 0) {
        for (uint32_t i = 0; i < available; i++) {
            uint8_t b = arg->comms->getInstream()[i];
            Serial.write(b);
        }
        arg->comms->consumeInputBytes(available);
    }
    xSemaphoreTake(arg->buffer_semaphore, portMAX_DELAY);
    for (;;) {
        while (Serial.available()) {
            arg->tx_buffer[arg->tx_buffer_top] = Serial.read();
            arg->tx_buffer_top++;
        }
        // Force a packet and terminate Serial broadcast
        // after 1200μs of inactivity on UART
        delayMicroseconds(1200);
        if (!Serial.available()) {
            break;
        }
    }
    arg->force_packet = true;
    xSemaphoreGive(arg->buffer_semaphore);
}

void do_comms_transmit_single(void* argptr) {
    ProtocolController* arg = (ProtocolController*)argptr;

    bool semaphore_taken = false;
    if (arg->tx_buffer_top > (arg->force_packet ? 0 : 16)) {
        xSemaphoreTake(arg->buffer_semaphore, portMAX_DELAY);
        semaphore_taken = true;
        if (arg->comms->getStatus() != COMMS_STATUS_OUTPUT) {
            arg->comms->startOutput();
        }
    }
    while (arg->tx_buffer_top > 16) {
        uint8_t prev_last_byte = arg->tx_buffer[16];
        arg->tx_buffer[16] = 1;
        arg->comms->transmit(arg->tx_buffer, 17);
        arg->tx_buffer[16] = prev_last_byte;
        // Consumption routine, shift queue down
        // TODO: Circular buffer
        for (uint32_t i = 16; i < arg->tx_buffer_top; i++) {
            arg->tx_buffer[i - 16] = arg->tx_buffer[i];
        }
        arg->tx_buffer_top -= 16;
    }
    if (arg->force_packet && arg->tx_buffer_top > 0) {
        uint8_t prev_last_byte = arg->tx_buffer[16];
        for (int i = arg->tx_buffer_top; i < 17; i++) {
            arg->tx_buffer[i] = 0;
        }
        arg->comms->transmit(arg->tx_buffer, 17);
        arg->tx_buffer[16] = prev_last_byte;
        arg->tx_buffer_top = 0;
        arg->comms->startInput();
    }
    if (semaphore_taken) {
        xSemaphoreGive(arg->buffer_semaphore);
    }

    delay(10);
}

void do_comms_transmit(void* argptr) {
    for (;;) { do_comms_transmit_single(argptr); }
}

ProtocolController::ProtocolController() {}

ProtocolController::ProtocolController(CommunicationSubsystem* comms, bool is_module) {
    this->doConstruction(comms, is_module);
}

void ProtocolController::doConstruction(CommunicationSubsystem* comms, bool is_module) {
    bool s = false;
    this->is_module = is_module;
    this->periodic_timer.timer_num = ResourceAllocatorPresets::hw_timer_ints.allocate(&s);
    this->comms = comms;
    void(*timer_isr)() = is_module ? &proto_timer_isr_module : &proto_timer_isr;
    if (s) {
        proto_timer_isr_ctx = (void*)this;
        periodic_timer.timer = timerBegin(periodic_timer.timer_num, 80, true);
        timerAttachInterrupt(periodic_timer.timer, timer_isr, true);
        timerWrite(periodic_timer.timer, 10000);
        timerAlarmEnable(periodic_timer.timer);
    }
    this->tx_buffer = (uint8_t*)malloc(PROTO_TX_BUFFER_LEN);
    this->tx_buffer_top = 0;
    this->buffer_semaphore = xSemaphoreCreateBinary();
    xTaskCreate(do_comms_transmit, "do_comms_transmit", 4096, (void*)this, 10, &this->dct_taskhandle);
    this->tenure();
    this->comms->startInput();
}

ProtocolController::~ProtocolController() {
    vTaskDelete(this->dct_taskhandle);
    timerAlarmDisable(periodic_timer.timer);
    timerDetachInterrupt(periodic_timer.timer);
    timerEnd(periodic_timer.timer);
    periodic_timer.timer = NULL;
    ResourceAllocatorPresets::hw_timer_ints.freeAllocation(periodic_timer.timer_num);
    vSemaphoreDelete(this->buffer_semaphore);
}