//override any HAL-specific macros
#pragma once
#include "bt_types.h"

//NFC_HAL_TASK=0 is already defined in gki_hal_target.h; it executes the Broadcom HAL
#define USERIAL_HAL_TASK  1  //execute userial's read thread
#define GKI_RUNNER_HAL_TASK 2  //execute GKI_run(), which runs forever
#define GKI_MAX_TASKS  3 //total of 3 tasks
