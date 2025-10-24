#pragma once
#include "esp_err.h"
#include "driver/i2c_master.h"

// Initialize I2C bus and run to dump registers
void run_i2c();
