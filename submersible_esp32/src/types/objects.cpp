#include "objects.h"
#include "primitives.h"
#include <setjmp.h>
#include "../resource_allocator.h"

#include "../objects/comms_subsystem.h"
#include "../objects/prop_subsystem.h"
#include "../objects/light_subsystem.h"
#include "../objects/proto_controller.h"

CREATE_EXC_SETUP(objects_constr);

// All object constructors must be specified here
CommunicationSubsystem::CommunicationSubsystem(BaseObject** params, uint8_t num_params):
    CommunicationSubsystem() {
    CHECK_TYPE_SIGNATURE(objects_constr, this, params, num_params, 0, );
}

PropulsionSubsystem::PropulsionSubsystem(BaseObject** params, uint8_t num_params):
    PropulsionSubsystem() {
    CHECK_TYPE_SIGNATURE(objects_constr, this, params, num_params, 0, );
}

LightingSubsystem::LightingSubsystem(BaseObject** params, uint8_t num_params):
    LightingSubsystem() {
    CHECK_TYPE_SIGNATURE(objects_constr, this, params, num_params, 0, );
}

ProtocolController::ProtocolController(BaseObject** params, uint8_t num_params):
    ProtocolController(NULL) {
    CHECK_TYPE_SIGNATURE(objects_constr, this, params, num_params, 1, TYPEID_COMMS_SUBSYS);
}

BaseObject* constructObject(uint16_t type_id, BaseObject** params, uint8_t num_params) {
#define ON_EXCEPTION(exc_data) { delete exc_data.object; }
    SETUP_EXC(objects_constr, ON_EXCEPTION);
#undef ON_EXCEPTION
    TRY_CONSTRUCT_OBJECT(CommunicationSubsystem, type_id, params, num_params);
    return NULL;
}