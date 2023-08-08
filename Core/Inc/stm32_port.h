/* Copyright 2020-2023 Espressif Systems (Shanghai) CO LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>
#include "esp_loader_io.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERIAL_FLASHER_BOOT_HOLD_TIME_MS 	1
#define SERIAL_FLASHER_RESET_HOLD_TIME_MS 	1
typedef struct {
    UART_HandleTypeDef *huart;
    GPIO_TypeDef *port_io0;
    uint16_t pin_num_io0;
    GPIO_TypeDef *port_rst;
    uint16_t pin_num_rst;
} loader_stm32_config_t;

void loader_port_stm32_init(loader_stm32_config_t *config);

#ifdef __cplusplus
}
#endif