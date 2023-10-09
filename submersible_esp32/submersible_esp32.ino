#include "src/registry.h"
#include "src/startup.h"

#define MAX_UNTENURED_AGE 2000

void setup() {
    runStartupProcedure();
}

void loop() {
    // Periodic
    mainObjectRegistry.runAllLivePeriodic();
    mainObjectRegistry.runGarbageCollection(MAX_UNTENURED_AGE);
}