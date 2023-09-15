#include "objects.h"
#include "primitives.h"
#include <setjmp.h>

CREATE_EXC_SETUP(objects_constr);

// All object constructors must be specified here

BaseObject* constructObject(uint16_t type_id, BaseObject** params, uint8_t num_params) {
    #define ON_EXCEPTION(exc_data) { delete exc_data.object; free(params); }
    SETUP_EXC(objects_constr, ON_EXCEPTION);
    #undef ON_EXCEPTION
    return NULL;
}