#include "startup.h"
#include "objects/comms_subsystem.h"
#include "objects/light_subsystem.h"
#include "objects/proto_controller.h"
#include "registry.h"
#include "Arduino.h"

void runStartupProcedure() {
    // Setup highest available CPU clock
    setCpuFrequencyMhz(240);

    // Initialize UART at 115200b
    Serial.begin(115200);
    Serial.println("SERIAL OK");

    // Initial lighting state off
    pinMode(4, OUTPUT);
    digitalWrite(4, LOW);

    // Bring up critical startup objects
    CommunicationSubsystem* comms = new CommunicationSubsystem();
    comms->tenure();
    mainObjectRegistry.addObject(comms);

    LightingSubsystem* lights = new LightingSubsystem();
    lights->tenure();
    mainObjectRegistry.addObject(lights);

    // Deduce module setting and bring up comms protocol
    pinMode(16, INPUT_PULLUP);
    ProtocolController* proto = new ProtocolController(comms, !!(digitalRead(16)));
    proto->tenure();
    mainObjectRegistry.addObject(proto);

    // System should now be idle, lights powered off, and awaiting hydroacoustic input.
}