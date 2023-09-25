#include "resource_allocator.h"

ResourceAllocator::ResourceAllocator(uint16_t max_allocations) {
    this->max_allocations = max_allocations;
    this->allocated = (uint8_t*)malloc((max_allocations + 7) / 8);
    memset(this->allocated, 0, (max_allocations + 7) / 8);
}

ResourceAllocator::~ResourceAllocator() {
    free(this->allocated);
}

uint16_t ResourceAllocator::allocate(bool* success) {
    for (uint16_t i = 0; i < max_allocations; i++) {
        if (!(allocated[i / 8] & (1 << (i & 7)))) {
            allocated[i / 8] |= (1 << (i & 7));
            *success = true;
            return i;
        }
    }
    *success = false;
    return UINT16_MAX;
}

void ResourceAllocator::freeAllocation(uint16_t ptr) {
    allocated[ptr / 8] &= ~(1 << (ptr & 7));
}

ResourceAllocator ResourceAllocatorPresets::hw_timer_ints(4);