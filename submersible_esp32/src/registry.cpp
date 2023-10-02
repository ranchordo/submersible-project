#include "registry.h"

ObjectRegistry::ObjectRegistry() {
    this->registry = (BaseObject**)malloc(8 * sizeof(BaseObject*));
    this->capacity = 8;
    for (uint16_t i = 0; i < this->capacity; i++) {
        this->registry[i] = NULL;
    }
}

ObjectRegistry::~ObjectRegistry() {
    free(this->registry);
}

uint16_t ObjectRegistry::addObject(BaseObject* obj) {
    if (obj == NULL) return UINT16_MAX;
    if (this->count >= this->capacity) {
        uint16_t new_capacity = this->capacity * 2;
        BaseObject** new_registry = (BaseObject**)malloc(new_capacity * sizeof(BaseObject*));
        for (uint16_t i = 0; i < new_capacity; i++) {
            if (i >= this->capacity) {
                new_registry[i] = NULL;
            }
            else {
                new_registry[i] = this->registry[i];
            }
        }
        free(this->registry);
        this->registry = new_registry;
        this->capacity = new_capacity;
    }
    for (uint16_t i = 0; i < this->capacity; i++) {
        if (this->registry[i] == NULL) {
            this->registry[i] = obj;
            this->count++;
            return i;
        }
    }
    return UINT16_MAX; // Very bad things
}

void ObjectRegistry::delObject(uint16_t handle) {
    delete this->registry[handle];
    this->registry[handle] = NULL;
    this->count--;
    if (this->count <= (this->capacity / 4)) {
        if (this->capacity >= 16) {
            uint16_t highest_handle = 0;
            for (uint16_t i = 0; i < capacity; i++) {
                if (this->registry[i] != NULL && i > highest_handle) {
                    highest_handle = i;
                }
            }
            if (highest_handle < (this->capacity / 2)) {
                // Then we can shrink
                uint16_t new_capacity = this->capacity / 2;
                BaseObject** new_registry = (BaseObject**)malloc(new_capacity * sizeof(BaseObject*));
                for (uint16_t i = 0; i < new_capacity; i++) {
                    new_registry[i] = this->registry[i];
                }
                free(this->registry);
                this->registry = new_registry;
                this->capacity = new_capacity;
            }
        }
    }
}

BaseObject* ObjectRegistry::getObject(uint16_t handle) {
    return this->registry[handle];
}

void ObjectRegistry::runGarbageCollection(unsigned long max_age_ms) {
    unsigned long time = millis();
    for (uint32_t i = 0; i < this->capacity; i++) {
        if (this->registry[i] != NULL) {
            if (this->registry[i]->creation_time_ms != UINT32_MAX) {
                if (time - this->registry[i]->creation_time_ms > max_age_ms) {
                    // Guhguhgarbage!
                    this->delObject(i);
                }
            }
        }
    }
}

bool ObjectRegistry::exists(uint16_t handle) {
    return (this->registry[handle] != NULL);
}

void ObjectRegistry::runAllLivePeriodic() {
    for (uint32_t i = 0; i < this->capacity; i++) {
        if (this->registry[i] != NULL) {
            if (this->registry[i]->enable_periodic) {
                this->registry[i]->periodic();
            }
        }
    }
}

ObjectRegistry mainObjectRegistry;