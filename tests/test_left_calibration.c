/* Preserve left-only branch coverage without changing the flashed configuration. */
#include "../application/chassis/chassis_config.h"
#undef CHASSIS_STEER_CALIBRATION_SIDE
#define CHASSIS_STEER_CALIBRATION_SIDE 1U
#define TEST_USE_CONFIG_SIDE 1
#include "test_dual_steer.c"
