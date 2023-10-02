#include "primitives.h"
#include <setjmp.h>

CREATE_EXC_SETUP(primitives_constr);

PInteger::PInteger(SerializationResult data) {
    if (data.datalen != 4) {
        throw_obj_exception(primitives_constr, this, "constr PInteger: size != 4");
        this->value = 0;
    } else {
        uint32_t bytes = 0;
        bytes += data.buffer[0];
        bytes += (data.buffer[1] << 8);
        bytes += (data.buffer[2] << 16);
        bytes += (data.buffer[3] << 24);
        this->value = *(reinterpret_cast<int*>(&bytes));
    }
}

PDouble::PDouble(SerializationResult data) {
    if (data.datalen != 8) {
        throw_obj_exception(primitives_constr, this, "constr PDouble: size != 8");
        this->value = 0.0;
    } else {
        uint64_t bytes = 0;
        bytes += data.buffer[0];
        bytes += (data.buffer[1] << 8);
        bytes += (data.buffer[2] << 16);
        bytes += (data.buffer[3] << 24);
        bytes += (data.buffer[4] << 32);
        bytes += (data.buffer[5] << 40);
        bytes += (data.buffer[6] << 48);
        bytes += (data.buffer[7] << 56);
        this->value = *(reinterpret_cast<double*>(&bytes));
    }
}

PString::PString(SerializationResult data) {
    char* cdata = reinterpret_cast<char*>(data.buffer);
    this->value = (char*)malloc(data.datalen);
    memcpy(this->value, cdata, data.datalen);
}

PString::PString(const char* value, bool is_exception) {
    this->is_exception = is_exception;
    size_t len = strlen(value);
    this->value = (char*)malloc(len);
    strncpy(this->value, value, len);
}

BaseObject* constructPrimitive(uint16_t type_id, SerializationResult data) {
    #define ON_EXCEPTION(exc_data) { delete exc_data.object; }
    SETUP_EXC(primitives_constr, ON_EXCEPTION);
    #undef ON_EXCEPTION
    
    TRY_CONSTRUCT_PRIMITIVE(PInteger, type_id, data);
    TRY_CONSTRUCT_PRIMITIVE(PDouble, type_id, data);
    TRY_CONSTRUCT_PRIMITIVE(PString, type_id, data);
    return NULL;
}