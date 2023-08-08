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

#define BIN_FIRST_SEGMENT_OFFSET    0x18
// Maximum block sized for RAM and Flash writes, respectively.
#define ESP_RAM_BLOCK               0x1800

#define MAX_CDC_BUFFER_SIZE			0x800

// For esp8266, esp32, esp32s2
#define BOOTLOADER_ADDRESS_V0       0x1000
// For esp32s3 and later chips
#define BOOTLOADER_ADDRESS_V1       0x0
#define PARTITION_ADDRESS           0x8000
#define APPLICATION_ADDRESS         0x10000

typedef struct {
    const uint8_t *data;
    uint32_t size;
    uint32_t addr;
} partition_attr_t;

typedef struct {
    partition_attr_t boot;
    partition_attr_t part;
    partition_attr_t app;
} example_binaries_t;

typedef struct {
    partition_attr_t ram_app;
} example_ram_app_binary_t;

typedef struct {
	uint8_t DO_Freq[4];
	uint8_t AO_Level;
	uint8_t Timeout;
} DIAI_Params_t;

typedef struct {
	uint8_t ONOFF;
	uint8_t Timeout;
} SIM_POWER_t;
/**
 * @brief esptool portable bin header format
 */
typedef struct example_bin_header {
    uint8_t magic;
    uint8_t segments;
    uint8_t flash_mode;
    uint8_t flash_size_freq;
    uint32_t entrypoint;
} example_bin_header_t;

/**
 * @brief esptool portable bin segment format
 */
typedef struct example_bin_segment {
    uint32_t addr;
    uint32_t size;
    uint8_t *data;
} example_bin_segment_t;


void get_example_binaries(target_chip_t target, example_binaries_t *binaries);
void get_example_ram_app_binary(target_chip_t target, example_ram_app_binary_t *bin);
esp_loader_error_t connect_to_target(uint32_t higher_transmission_rate);
esp_loader_error_t flash_binary(const uint8_t *bin, size_t size, size_t address);
esp_loader_error_t load_ram_binary(const uint8_t *bin);


typedef enum {
	IDLE,
	POWER_ON,
	POWER_OFF,
	CHECK_CURRENT,
	CHECK_5V,
	CHECK_4V2,
	CHECK_3V3,
	CONNECT_ESP,
	START_FLASH_BOOTLOADER,
	START_FLASH_PARTITION,
	START_FLASH_APP,
	RECEIVE_BIN,
	RESET_ESP,
//	RECEIVE_BIN_PARTITIONS,
//	FLASH_ESP,

	CHECK_RS485,
	CHECK_RS232,
	GET_TEMPERATE,
	GET_BATTERY_CAPACITY,
	CHECK_AIDI,
	CHECK_1V8,
	CHECK_SIM_STATUS,
	CHECK_SIM_SIGNAL,
	CHECK_SIM_ETH,
	CHECK_ETH,
	CHECK_LED,
	CHECK_RELAY
} SM_TEST_Step;
