// RAM state (see fw_state.h).
#include "fw_state.h"

#include <string.h>

namespace ubts_fw {

FwState fw;

void state_reset() { memset(&fw, 0, sizeof fw); }

}  // namespace ubts_fw
