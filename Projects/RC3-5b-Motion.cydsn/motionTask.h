#pragma once

#include "event_groups.h"

extern EventGroupHandle_t systemInputMode;
#define MODE_MOTION (1<<0)
#define MODE_CAPSENSE (1<<1)

void motionTask(void *arg);
