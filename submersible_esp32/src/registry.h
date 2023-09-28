#pragma once

#include <Arduino.h>
#include "types/baseobject.h"

class ObjectRegistry {
public:
    ObjectRegistry();
    ~ObjectRegistry();
    uint16_t addObject(BaseObject* obj);
    void delObject(uint16_t handle);
    BaseObject* getObject(uint16_t handle);
    bool exists(uint16_t handle);

    void runGarbageCollection(unsigned long max_age_ms);
    void runAllLivePeriodic();

    uint16_t getNumLiveObjects() {
        return this->count;
    }
    uint16_t getCurrentRegistryCapacity() {
        return this->capacity;
    }
    BaseObject** getInternalRegistry() {
        return this->registry;
    }
private:
    BaseObject** registry = NULL;
    uint16_t capacity = 0;
    uint16_t count = 0;
};

extern ObjectRegistry mainObjectRegistry;