#pragma once
#include <Arduino.h>

class ResourceAllocator {
public:
    ResourceAllocator(uint16_t max_allocations);
    ~ResourceAllocator();
    uint16_t allocate(bool* success);
    void freeAllocation(uint16_t ptr);
private:
    uint8_t* allocated;
    uint16_t max_allocations;
};

class ResourceAllocatorPresets {
public:
    static ResourceAllocator hw_timer_ints;
    static ResourceAllocator ledc_channels;
};