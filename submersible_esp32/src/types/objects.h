#pragma once

#include "baseobject.h"

enum ObjectTypeId {
    TYPEID_COMMS_SUBSYS = 0,
};

#define CHECK_TYPE_SIGNATURE(name, object, params, num_params, num_expected, expected...) \
{ \
    uint16_t exp[num_expected] = {expected}; \
    if (num_params != num_expected) { \
        throw_obj_exception(name, object, "Arities don't match"); \
    } \
    for (uint8_t i = 0; i < num_expected; i++) { \
        if (params[i]->getTypeId() != exp[i]) { \
            throw_obj_exception(name, object, "Type signatures don't match"); \
        } \
    } \
}

extern BaseObject* constructObject(uint16_t type_id, BaseObject** params, uint8_t num_params);