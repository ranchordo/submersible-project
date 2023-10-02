#pragma once
#include "../types/objects.h"
#include "comms_subsystem.h"

#define PROTO_TX_BUFFER_LEN 128

class ProtocolController: public BaseObject {
public:
    ProtocolController(CommunicationSubsystem* comms);
    ProtocolController(BaseObject** params, uint8_t num_params);
    ~ProtocolController();
    SerializationResult serialize() override {
        return { NULL, 0 };
    }
    uint16_t getTypeId() override { return TYPEID_PROTO_CONTROLLER; }
    static uint16_t _static_typeId() { return TYPEID_PROTO_CONTROLLER; }
    const char* getTypeString() override { return "Proto_Ctrl"; }
    BaseObject* callMethod(uint8_t slot, BaseObject** params, uint8_t num_params) override;

private:
    struct {
        uint16_t timer_num;
        hw_timer_t* timer;
    } periodic_timer;
    CommunicationSubsystem* comms;
    uint8_t* tx_buffer;
    volatile uint32_t tx_buffer_top;
    volatile bool force_packet = false;
    SemaphoreHandle_t buffer_semaphore;
    friend void proto_timer_isr();
    friend void do_comms_transmit(void*);
};