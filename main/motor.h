#pragma once

#include <stdbool.h>
#include "esp_err.h"

esp_err_t motor_init(void);

void motor_set(float left, float right, bool armed);

void motor_brake(void);
