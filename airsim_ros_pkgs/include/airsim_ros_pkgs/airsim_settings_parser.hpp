#include <chrono>
#include <iostream>

// Airsim library headers
#include "common/AirSimSettings.hpp"
#include "common/common_utils/FileSystem.hpp"
#include "common/common_utils/StrictMode.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON