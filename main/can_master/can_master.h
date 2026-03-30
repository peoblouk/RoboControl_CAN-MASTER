#ifndef CAN_MASTER_H
#define CAN_MASTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "can_master_protocol.h"

// Stamp-C3U + MCP2515 defaults.
#define CAN_MASTER_SPI_HOST            SPI2_HOST
#define CAN_MASTER_SPI_SCK_GPIO        GPIO_NUM_4
#define CAN_MASTER_SPI_MISO_GPIO       GPIO_NUM_5
#define CAN_MASTER_SPI_MOSI_GPIO       GPIO_NUM_6
#define CAN_MASTER_SPI_CS_GPIO         GPIO_NUM_7
#define CAN_MASTER_INT_GPIO            GPIO_NUM_10
// Start conservative to tolerate long wires and level shifters; you can raise later.
#define CAN_MASTER_SPI_CLOCK_HZ        (1 * 1000 * 1000)

// MCP2515 oscillator frequency on your module (common values: 8 MHz or 16 MHz).
#define CAN_MASTER_MCP2515_OSC_HZ      8000000
#define CAN_MASTER_BITRATE             500000

#define CAN_MASTER_NODE1_ID            0x01
#define CAN_MASTER_NODE2_ID            0x02

#define CAN_MASTER_RESPONSE_TIMEOUT_MS 800
#define CAN_MASTER_NODE_ONLINE_TIMEOUT_MS 1500

typedef struct {
    uint8_t node_id;
    uint8_t cmd;
    can_protocol_result_t result;
    uint8_t robot_state;
    uint8_t state_flags;
    uint8_t detail[4];
} can_master_response_t;

typedef struct {
    uint8_t node_id;
    uint8_t protocol_version;
    uint8_t robot_state;
    uint8_t state_flags;
    uint8_t prepared_slot;
    uint8_t active_slot;
    can_protocol_result_t last_protocol_error;
    uint8_t last_robot_error;
} can_master_status_t;

typedef struct {
    uint8_t canstat;
    uint8_t canctrl;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
    uint8_t caninte;
    uint8_t canintf;
    uint8_t eflg;
    uint8_t txb0ctrl;
    uint8_t rxb0ctrl;
    uint8_t rxb1ctrl;
} can_master_probe_t;

typedef struct {
    uint32_t tx_id;
    uint32_t rx_id;
    uint8_t dlc;
    uint8_t tx_data[8];
    uint8_t rx_data[8];
    bool matched;
} can_master_loopback_result_t;

esp_err_t can_master_init(void);
bool can_master_is_ready(void);

esp_err_t can_master_arm(uint8_t node_id, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_disarm(uint8_t node_id, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_home(uint8_t node_id, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_prepare_slot(uint8_t node_id, uint8_t slot, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_program_run(uint8_t node_id, uint8_t slot, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_program_delete(uint8_t node_id, uint8_t slot, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_request_status(uint8_t node_id, can_master_response_t *out_response, uint32_t timeout_ms);
esp_err_t can_master_stop_all(void);
esp_err_t can_master_sync_start_all(void);

esp_err_t can_master_upload_nc_program(uint8_t node_id,
                                       uint8_t slot,
                                       const uint8_t *program_bytes,
                                       size_t size_bytes,
                                       uint32_t timeout_ms);

esp_err_t can_master_get_last_status(uint8_t node_id, can_master_status_t *out_status);
bool can_master_node_is_online(uint8_t node_id, uint32_t max_age_ms);
size_t can_master_count_online_nodes(const uint8_t *node_ids, size_t node_count, uint32_t max_age_ms);
esp_err_t can_master_debug_probe(can_master_probe_t *out_probe);
esp_err_t can_master_debug_loopback(can_master_loopback_result_t *out_result);
const char *can_master_protocol_result_to_str(can_protocol_result_t code);

#endif // CAN_MASTER_H
