#include "can_master.h"

#include <inttypes.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "can_master";

#define RESP_QUEUE_LEN 48
#define RX_TASK_STACK 4096
#define RX_TASK_PRIO 8
#define RX_POLL_MS 2
#define RX_DRAIN_MAX_ITERS 32
#define SPI_LOCK_MS 1000
#define TX_WAIT_MS 50
#define STATUS_CACHE_NODE_COUNT 128
#define INFO_CACHE_NODE_COUNT 128
#define INFO_MASK_WORK_OFFSET  (1U << 0)
#define INFO_MASK_META         (1U << 1)
#define INFO_MASK_VALUES_0_2   (1U << 2)
#define INFO_MASK_VALUES_3_5   (1U << 3)
#define INFO_MASK_SENSORS      (INFO_MASK_META | INFO_MASK_VALUES_0_2 | INFO_MASK_VALUES_3_5)

#define MCP_RESET 0xC0
#define MCP_READ 0x03
#define MCP_WRITE 0x02
#define MCP_BIT_MODIFY 0x05
#define MCP_RTS 0x80

#define REG_TXRTSCTRL 0x0D
#define REG_CANSTAT 0x0E
#define REG_CANCTRL 0x0F
#define REG_CNF3 0x28
#define REG_CNF2 0x29
#define REG_CNF1 0x2A
#define REG_CANINTE 0x2B
#define REG_CANINTF 0x2C
#define REG_EFLG 0x2D
#define REG_TXB0CTRL 0x30
#define REG_TXB0SIDH 0x31
#define REG_RXB0CTRL 0x60
#define REG_RXB0SIDH 0x61
#define REG_RXB1CTRL 0x70
#define REG_RXB1SIDH 0x71

#define CANCTRL_REQOP_MASK 0xE0
#define CANCTRL_REQOP_NORMAL 0x00
#define CANCTRL_REQOP_LOOPBACK 0x40
#define CANCTRL_REQOP_CONFIG 0x80
#define CANCTRL_ABAT 0x10

#define CANINTE_RX0IE 0x01
#define CANINTE_RX1IE 0x02
#define CANINTE_ERRIE 0x20

#define CANINTF_RX0IF 0x01
#define CANINTF_RX1IF 0x02
#define CANINTF_TX0IF 0x04
#define CANINTF_ERRIF 0x20

#define EFLG_RX1OVR 0x80
#define EFLG_RX0OVR 0x40
#define EFLG_TXBO 0x20
#define EFLG_TXEP 0x10
#define EFLG_RXEP 0x08

#define TXBCTRL_ABTF 0x40
#define TXBCTRL_MLOA 0x20
#define TXBCTRL_TXERR 0x10
#define TXBCTRL_TXREQ 0x08

#define SIDL_EXIDE 0x08

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
} mcp_frame_t;

typedef struct {
    uint8_t node_id;
    bool valid;
    TickType_t last_seen_tick;
    can_master_status_t status;
} status_cache_entry_t;

typedef struct {
    uint8_t node_id;
    bool valid;
    TickType_t last_seen_tick;
    uint8_t frame_mask;
    can_master_work_offset_t work_offset;
    can_master_sensor_values_t sensors;
} info_cache_entry_t;

typedef struct {
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp_bt_t;

static volatile bool s_ready = false;
static spi_device_handle_t s_spi = NULL;
static SemaphoreHandle_t s_spi_lock = NULL;
static QueueHandle_t s_resp_q = NULL;
static TaskHandle_t s_rx_task = NULL;
static status_cache_entry_t s_status[STATUS_CACHE_NODE_COUNT] = {0};
static portMUX_TYPE s_status_mux = portMUX_INITIALIZER_UNLOCKED;
static info_cache_entry_t s_info[INFO_CACHE_NODE_COUNT] = {0};
static portMUX_TYPE s_info_mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR can_int_isr(void *arg)
{
    (void)arg;
    BaseType_t higher_priority_task_woken = pdFALSE;
    if (s_rx_task != NULL) {
        vTaskNotifyGiveFromISR(s_rx_task, &higher_priority_task_woken);
    }
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static const char *cmd_to_str(uint8_t cmd)
{
    switch ((can_command_id_t)cmd) {
        case CAN_CMD_GET_STATUS: return "GET_STATUS";
        case CAN_CMD_ARM: return "ARM";
        case CAN_CMD_DISARM: return "DISARM";
        case CAN_CMD_HOME: return "HOME";
        case CAN_CMD_STOP: return "STOP";
        case CAN_CMD_UPLOAD_BEGIN: return "UPLOAD_BEGIN";
        case CAN_CMD_UPLOAD_DATA: return "UPLOAD_DATA";
        case CAN_CMD_UPLOAD_END: return "UPLOAD_END";
        case CAN_CMD_PROGRAM_RUN: return "PROGRAM_RUN";
        case CAN_CMD_PROGRAM_DELETE: return "PROGRAM_DELETE";
        case CAN_CMD_PREPARE: return "PREPARE";
        case CAN_CMD_SYNC_START: return "SYNC_START";
        default: return "UNKNOWN";
    }
}

const char *can_master_protocol_result_to_str(can_protocol_result_t code)
{
    switch (code) {
        case CAN_PROTO_OK: return "ok";
        case CAN_PROTO_ERR_INVALID_CMD: return "invalid_cmd";
        case CAN_PROTO_ERR_INVALID_LENGTH: return "invalid_length";
        case CAN_PROTO_ERR_INVALID_SLOT: return "invalid_slot";
        case CAN_PROTO_ERR_UPLOAD_STATE: return "upload_state";
        case CAN_PROTO_ERR_SEQUENCE: return "sequence";
        case CAN_PROTO_ERR_BUSY: return "busy";
        case CAN_PROTO_ERR_NOT_READY: return "not_ready";
        case CAN_PROTO_ERR_FILE: return "file";
        case CAN_PROTO_ERR_EXEC: return "exec";
        default: return "unknown";
    }
}

static bool node_valid(uint8_t node_id) { return node_id > 0 && node_id <= CAN_NODE_ID_MAX; }

static bool is_response_id(uint32_t id, uint8_t *node)
{
    if (id < CAN_RESP_ID_BASE || id > (CAN_RESP_ID_BASE + CAN_NODE_ID_MAX)) return false;
    if (node != NULL) *node = (uint8_t)(id - CAN_RESP_ID_BASE);
    return true;
}

static bool is_status_id(uint32_t id, uint8_t *node)
{
    if (id < CAN_STATUS_ID_BASE || id > (CAN_STATUS_ID_BASE + CAN_NODE_ID_MAX)) return false;
    if (node != NULL) *node = (uint8_t)(id - CAN_STATUS_ID_BASE);
    return true;
}

static bool is_info_id(uint32_t id, uint8_t *node)
{
    if (id < CAN_INFO_ID_BASE || id > (CAN_INFO_ID_BASE + CAN_NODE_ID_MAX)) return false;
    if (node != NULL) *node = (uint8_t)(id - CAN_INFO_ID_BASE);
    return true;
}

static uint32_t ticks_to_ms(TickType_t ticks)
{
    uint64_t ms = (uint64_t)ticks * (uint64_t)portTICK_PERIOD_MS;
    return (ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)ms;
}

static int16_t decode_i16_le(const uint8_t *data)
{
    if (data == NULL) return 0;
    return (int16_t)(((uint16_t)data[1] << 8U) | (uint16_t)data[0]);
}

static bool lock_spi(void)
{
    return (s_spi_lock != NULL) && (xSemaphoreTake(s_spi_lock, pdMS_TO_TICKS(SPI_LOCK_MS)) == pdTRUE);
}

static void unlock_spi(void)
{
    if (s_spi_lock != NULL) xSemaphoreGive(s_spi_lock);
}

static esp_err_t spi_xfer(const uint8_t *tx, uint8_t *rx, size_t len)
{
    if (s_spi == NULL || tx == NULL || len == 0) return ESP_ERR_INVALID_ARG;
    spi_transaction_t t = {0};
    t.length = len * 8U;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    return spi_device_transmit(s_spi, &t);
}

static esp_err_t mcp_reset_locked(void)
{
    const uint8_t cmd[1] = {MCP_RESET};
    return spi_xfer(cmd, NULL, sizeof(cmd));
}

static esp_err_t mcp_read_reg_locked(uint8_t reg, uint8_t *val)
{
    if (val == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t tx[3] = {MCP_READ, reg, 0};
    uint8_t rx[3] = {0};
    esp_err_t err = spi_xfer(tx, rx, sizeof(tx));
    if (err != ESP_OK) return err;
    *val = rx[2];
    return ESP_OK;
}

static esp_err_t mcp_write_reg_locked(uint8_t reg, uint8_t val)
{
    uint8_t tx[3] = {MCP_WRITE, reg, val};
    return spi_xfer(tx, NULL, sizeof(tx));
}

static esp_err_t mcp_read_regs_locked(uint8_t reg, uint8_t *out, size_t len)
{
    if (out == NULL || len == 0 || len > 32) return ESP_ERR_INVALID_ARG;
    uint8_t tx[34] = {0};
    uint8_t rx[34] = {0};
    tx[0] = MCP_READ;
    tx[1] = reg;
    esp_err_t err = spi_xfer(tx, rx, len + 2U);
    if (err != ESP_OK) return err;
    memcpy(out, &rx[2], len);
    return ESP_OK;
}

static esp_err_t mcp_write_regs_locked(uint8_t reg, const uint8_t *in, size_t len)
{
    if (in == NULL || len == 0 || len > 32) return ESP_ERR_INVALID_ARG;
    uint8_t tx[34] = {0};
    tx[0] = MCP_WRITE;
    tx[1] = reg;
    memcpy(&tx[2], in, len);
    return spi_xfer(tx, NULL, len + 2U);
}

static esp_err_t mcp_bit_modify_locked(uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t tx[4] = {MCP_BIT_MODIFY, reg, mask, val};
    return spi_xfer(tx, NULL, sizeof(tx));
}

static esp_err_t mcp_rts_locked(uint8_t mask)
{
    uint8_t tx[1] = {(uint8_t)(MCP_RTS | (mask & 0x07U))};
    return spi_xfer(tx, NULL, sizeof(tx));
}

static esp_err_t mcp_set_mode_locked(uint8_t mode)
{
    esp_err_t err = mcp_bit_modify_locked(REG_CANCTRL, CANCTRL_REQOP_MASK, mode);
    if (err != ESP_OK) return err;

    for (int i = 0; i < 50; i++) {
        uint8_t canstat = 0;
        err = mcp_read_reg_locked(REG_CANSTAT, &canstat);
        if (err != ESP_OK) return err;
        if ((canstat & CANCTRL_REQOP_MASK) != mode && i == 0) {
            ESP_LOGW(TAG, "CANSTAT=0x%02X (waiting for mode=0x%02X)", canstat, mode);
        }
        if ((canstat & CANCTRL_REQOP_MASK) == mode) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return ESP_ERR_TIMEOUT;
}

static bool mcp_calc_bt(uint32_t osc_hz, uint32_t bitrate, mcp_bt_t *out)
{
    if (out == NULL || osc_hz == 0 || bitrate == 0) return false;

    bool found = false;
    int best_score = INT_MAX;
    mcp_bt_t best = {0};

    for (int tseg2 = 2; tseg2 <= 8; tseg2++) {
        for (int tseg1 = 2; tseg1 <= 16; tseg1++) {
            int nbt = 1 + tseg1 + tseg2;
            uint64_t den = 2ULL * (uint64_t)bitrate * (uint64_t)nbt;
            if (den == 0 || ((uint64_t)osc_hz % den) != 0ULL) continue;
            uint32_t brp1 = (uint32_t)((uint64_t)osc_hz / den);
            if (brp1 == 0 || brp1 > 64U) continue;

            int prop = tseg1 / 2;
            if (prop < 1) prop = 1;
            if (prop > 8) prop = 8;
            int ph1 = tseg1 - prop;
            if (ph1 < 1) ph1 = 1;
            if (ph1 > 8) ph1 = 8;
            prop = tseg1 - ph1;
            if (prop < 1 || prop > 8) continue;

            int sample_pm = ((1 + tseg1) * 1000) / nbt;
            int score = abs(sample_pm - 800);
            if (!found || score < best_score) {
                found = true;
                best_score = score;
                best.cnf1 = (uint8_t)(brp1 - 1U);
                best.cnf2 = (uint8_t)(0x80U | ((uint8_t)(ph1 - 1) << 3U) | (uint8_t)(prop - 1));
                best.cnf3 = (uint8_t)(tseg2 - 1);
            }
        }
    }

    if (!found) return false;
    *out = best;
    return true;
}

static esp_err_t mcp_configure_locked(void)
{
    mcp_bt_t bt = {0};
    if (!mcp_calc_bt(CAN_MASTER_MCP2515_OSC_HZ, CAN_MASTER_BITRATE, &bt)) {
        ESP_LOGE(TAG,
                 "No bit timing for osc=%" PRIu32 " bitrate=%" PRIu32,
                 (uint32_t)CAN_MASTER_MCP2515_OSC_HZ,
                 (uint32_t)CAN_MASTER_BITRATE);
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "MCP2515 timing CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X", bt.cnf1, bt.cnf2, bt.cnf3);

    esp_err_t err = mcp_write_reg_locked(REG_CNF1, bt.cnf1);
    if (err != ESP_OK) return err;
    err = mcp_write_reg_locked(REG_CNF2, bt.cnf2);
    if (err != ESP_OK) return err;
    err = mcp_write_reg_locked(REG_CNF3, bt.cnf3);
    if (err != ESP_OK) return err;

    err = mcp_write_reg_locked(REG_TXRTSCTRL, 0x00); // disable floating RTS pins
    if (err != ESP_OK) return err;

    err = mcp_write_reg_locked(REG_RXB0CTRL, 0x64); // accept all + rollover
    if (err != ESP_OK) return err;
    err = mcp_write_reg_locked(REG_RXB1CTRL, 0x60); // accept all
    if (err != ESP_OK) return err;

    err = mcp_write_reg_locked(REG_CANINTE, CANINTE_RX0IE | CANINTE_RX1IE | CANINTE_ERRIE);
    if (err != ESP_OK) return err;
    err = mcp_write_reg_locked(REG_CANINTF, 0x00);
    if (err != ESP_OK) return err;
    err = mcp_write_reg_locked(REG_EFLG, 0x00);
    if (err != ESP_OK) return err;
    return ESP_OK;
}

static esp_err_t mcp_read_frame_locked(bool use_rxb1, mcp_frame_t *f)
{
    if (f == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t raw[13] = {0};
    uint8_t base = use_rxb1 ? REG_RXB1SIDH : REG_RXB0SIDH;
    esp_err_t err = mcp_read_regs_locked(base, raw, sizeof(raw));
    if (err != ESP_OK) return err;

    uint8_t sidh = raw[0];
    uint8_t sidl = raw[1];
    if ((sidl & SIDL_EXIDE) != 0) return ESP_ERR_NOT_SUPPORTED;

    f->id = ((uint32_t)sidh << 3U) | ((uint32_t)sidl >> 5U);
    f->dlc = (uint8_t)(raw[4] & 0x0FU);
    if (f->dlc > 8U) f->dlc = 8U;
    memcpy(f->data, &raw[5], f->dlc);
    return ESP_OK;
}

static esp_err_t mcp_send_std_locked(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (id > 0x7FFU || dlc > 8U) return ESP_ERR_INVALID_ARG;

    uint8_t txb0ctrl = 0;
    esp_err_t err = mcp_read_reg_locked(REG_TXB0CTRL, &txb0ctrl);
    if (err != ESP_OK) return err;
    if (txb0ctrl & TXBCTRL_TXREQ) return ESP_ERR_INVALID_STATE;

    uint8_t frame[13] = {0};
    frame[0] = (uint8_t)(id >> 3U);
    frame[1] = (uint8_t)((id & 0x07U) << 5U);
    frame[2] = 0;
    frame[3] = 0;
    frame[4] = (uint8_t)(dlc & 0x0FU);
    if (dlc > 0 && data != NULL) memcpy(&frame[5], data, dlc);

    err = mcp_write_regs_locked(REG_TXB0SIDH, frame, sizeof(frame));
    if (err != ESP_OK) return err;

    err = mcp_bit_modify_locked(REG_CANINTF, CANINTF_TX0IF, 0x00);
    if (err != ESP_OK) return err;

    err = mcp_rts_locked(0x01);
    if (err != ESP_OK) return err;

    for (int i = 0; i < TX_WAIT_MS; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        txb0ctrl = 0;
        err = mcp_read_reg_locked(REG_TXB0CTRL, &txb0ctrl);
        if (err != ESP_OK) return err;
        if ((txb0ctrl & TXBCTRL_TXREQ) == 0) {
            (void)mcp_bit_modify_locked(REG_CANINTF, CANINTF_TX0IF, 0x00);
            if (txb0ctrl & (TXBCTRL_ABTF | TXBCTRL_MLOA | TXBCTRL_TXERR)) return ESP_FAIL;
            return ESP_OK;
        }
    }

    (void)mcp_bit_modify_locked(REG_CANCTRL, CANCTRL_ABAT, CANCTRL_ABAT);
    (void)mcp_bit_modify_locked(REG_CANCTRL, CANCTRL_ABAT, 0x00);
    return ESP_ERR_TIMEOUT;
}

static void cache_status(const can_master_status_t *st)
{
    if (st == NULL || !node_valid(st->node_id)) return;
    portENTER_CRITICAL(&s_status_mux);
    s_status[st->node_id].valid = true;
    s_status[st->node_id].node_id = st->node_id;
    s_status[st->node_id].last_seen_tick = xTaskGetTickCount();
    s_status[st->node_id].status = *st;
    portEXIT_CRITICAL(&s_status_mux);
}

static void info_cache_reset(info_cache_entry_t *entry, uint8_t node_id)
{
    if (entry == NULL) return;

    memset(entry, 0, sizeof(*entry));
    entry->node_id = node_id;
    entry->work_offset.node_id = node_id;
    entry->sensors.node_id = node_id;
    for (size_t i = 0; i < 3U; i++) {
        entry->work_offset.work_offset_0p1mm[i] = CAN_INFO_VALUE_INVALID_I16;
    }
    for (size_t i = 0; i < 6U; i++) {
        entry->sensors.values_0p1deg[i] = CAN_INFO_VALUE_INVALID_I16;
    }
}

static void clear_info_cache(uint8_t node_id)
{
    if (!node_valid(node_id)) return;

    portENTER_CRITICAL(&s_info_mux);
    info_cache_reset(&s_info[node_id], node_id);
    portEXIT_CRITICAL(&s_info_mux);
}

static void handle_info(uint8_t node, const mcp_frame_t *f)
{
    if (!node_valid(node) || f == NULL || f->dlc < 2U) return;

    portENTER_CRITICAL(&s_info_mux);
    info_cache_entry_t *entry = &s_info[node];
    if (!entry->valid) {
        info_cache_reset(entry, node);
        entry->valid = true;
    }

    entry->last_seen_tick = xTaskGetTickCount();

    switch ((can_info_frame_kind_t)f->data[0]) {
        case CAN_INFO_WORK_OFFSET:
            if (f->dlc >= 7U) {
                entry->work_offset.work_offset_0p1mm[0] = decode_i16_le(&f->data[1]);
                entry->work_offset.work_offset_0p1mm[1] = decode_i16_le(&f->data[3]);
                entry->work_offset.work_offset_0p1mm[2] = decode_i16_le(&f->data[5]);
                entry->frame_mask |= INFO_MASK_WORK_OFFSET;
            }
            break;

        case CAN_INFO_TCP_META:
            if (f->dlc >= 8U) {
                entry->sensors.value_source = (can_info_value_source_t)f->data[4];
                entry->sensors.value_count = f->data[5];
                entry->frame_mask |= INFO_MASK_META;
            }
            break;

        case CAN_INFO_VALUES_0_2:
            if (f->dlc >= 8U) {
                entry->sensors.values_0p1deg[0] = decode_i16_le(&f->data[1]);
                entry->sensors.values_0p1deg[1] = decode_i16_le(&f->data[3]);
                entry->sensors.values_0p1deg[2] = decode_i16_le(&f->data[5]);
                entry->sensors.value_count = f->data[7];
                entry->frame_mask |= INFO_MASK_VALUES_0_2;
            }
            break;

        case CAN_INFO_VALUES_3_5:
            if (f->dlc >= 8U) {
                entry->sensors.values_0p1deg[3] = decode_i16_le(&f->data[1]);
                entry->sensors.values_0p1deg[4] = decode_i16_le(&f->data[3]);
                entry->sensors.values_0p1deg[5] = decode_i16_le(&f->data[5]);
                entry->sensors.value_count = f->data[7];
                entry->frame_mask |= INFO_MASK_VALUES_3_5;
            }
            break;

        default:
            break;
    }
    portEXIT_CRITICAL(&s_info_mux);
}

static void handle_status(uint8_t node, const mcp_frame_t *f)
{
    if (f->dlc < 8U) return;
    can_master_status_t st = {
        .node_id = node,
        .protocol_version = f->data[0],
        .robot_state = f->data[1],
        .state_flags = f->data[2],
        .prepared_slot = f->data[3],
        .active_slot = f->data[4],
        .last_protocol_error = (can_protocol_result_t)f->data[5],
        .last_robot_error = f->data[6],
    };
    cache_status(&st);
}

static void handle_response(uint8_t node, const mcp_frame_t *f)
{
    if (f->dlc < 4U) return;

    can_master_response_t r = {
        .node_id = node,
        .cmd = f->data[0],
        .result = (can_protocol_result_t)f->data[1],
        .robot_state = f->data[2],
        .state_flags = f->data[3],
        .detail = {0, 0, 0, 0},
    };
    for (uint8_t i = 0; i < 4U && (4U + i) < f->dlc; i++) r.detail[i] = f->data[4U + i];

    if (xQueueSend(s_resp_q, &r, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Response queue full, dropping node=%u cmd=%s", node, cmd_to_str(r.cmd));
    }
}

static void handle_frame(const mcp_frame_t *f)
{
    if (f == NULL) return;
    uint8_t node = 0;
    if (is_response_id(f->id, &node)) {
        handle_response(node, f);
        return;
    }
    if (is_status_id(f->id, &node)) {
        handle_status(node, f);
        return;
    }
    if (is_info_id(f->id, &node)) {
        handle_info(node, f);
        return;
    }
}

static void rx_drain_pending_locked(void)
{
    for (int iter = 0; iter < RX_DRAIN_MAX_ITERS; iter++) {
        uint8_t intf = 0;
        esp_err_t err = mcp_read_reg_locked(REG_CANINTF, &intf);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RX drain read CANINTF failed: %s", esp_err_to_name(err));
            return;
        }

        bool did_work = false;

        if (intf & CANINTF_RX0IF) {
            mcp_frame_t frame = {0};
            if (mcp_read_frame_locked(false, &frame) == ESP_OK) {
                handle_frame(&frame);
            }
            (void)mcp_bit_modify_locked(REG_CANINTF, CANINTF_RX0IF, 0x00);
            did_work = true;
        }

        if (intf & CANINTF_RX1IF) {
            mcp_frame_t frame = {0};
            if (mcp_read_frame_locked(true, &frame) == ESP_OK) {
                handle_frame(&frame);
            }
            (void)mcp_bit_modify_locked(REG_CANINTF, CANINTF_RX1IF, 0x00);
            did_work = true;
        }

        if (intf & CANINTF_ERRIF) {
            uint8_t eflg = 0;
            (void)mcp_read_reg_locked(REG_EFLG, &eflg);
            ESP_LOGW(TAG, "MCP2515 ERRIF eflg=0x%02X (TXBO=%u TXEP=%u RXEP=%u RXOVR=%u)",
                     (unsigned)eflg,
                     (unsigned)((eflg & EFLG_TXBO) != 0),
                     (unsigned)((eflg & EFLG_TXEP) != 0),
                     (unsigned)((eflg & EFLG_RXEP) != 0),
                     (unsigned)((eflg & (EFLG_RX0OVR | EFLG_RX1OVR)) != 0));
            (void)mcp_bit_modify_locked(REG_EFLG, (EFLG_RX0OVR | EFLG_RX1OVR), 0x00);
            (void)mcp_bit_modify_locked(REG_CANINTF, CANINTF_ERRIF, 0x00);
            did_work = true;
        }

        if (!did_work) {
            return;
        }
    }

    ESP_LOGW(TAG, "RX drain reached iteration limit");
}

static void rx_task(void *arg)
{
    (void)arg;
    const TickType_t wait_ticks = pdMS_TO_TICKS(RX_POLL_MS);
    const TickType_t wait_timeout = (wait_ticks > 0) ? wait_ticks : 1;

    for (;;) {
        if (!s_ready) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        bool should_service = (gpio_get_level(CAN_MASTER_INT_GPIO) == 0);
        if (!should_service) {
            should_service = (ulTaskNotifyTake(pdTRUE, wait_timeout) > 0U);
            if (!should_service) {
                should_service = (gpio_get_level(CAN_MASTER_INT_GPIO) == 0);
            }
        }

        if (!should_service) continue;
        if (!lock_spi()) continue;
        rx_drain_pending_locked();
        unlock_spi();
    }
}

static esp_err_t send_frame(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    if (!s_ready) return ESP_ERR_INVALID_STATE;
    if (id > 0x7FFU || dlc > 8U) return ESP_ERR_INVALID_ARG;

    if (!lock_spi()) return ESP_ERR_TIMEOUT;
    esp_err_t err = mcp_send_std_locked(id, data, dlc);
    unlock_spi();

    if (err != ESP_OK) ESP_LOGW(TAG, "TX failed id=0x%03" PRIX32 " err=%s", id, esp_err_to_name(err));
    return err;
}

static esp_err_t send_cmd_raw(uint32_t id, can_command_id_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > 7U) return ESP_ERR_INVALID_ARG;

    uint8_t frame[8] = {0};
    frame[0] = (uint8_t)cmd;
    if (payload_len > 0 && payload != NULL) memcpy(&frame[1], payload, payload_len);

    ESP_LOGI(TAG, "TX id=0x%03" PRIX32 " cmd=%s(%02X) dlc=%u", id, cmd_to_str((uint8_t)cmd), (unsigned)cmd, (unsigned)(payload_len + 1U));
    return send_frame(id, frame, (uint8_t)(payload_len + 1U));
}

static esp_err_t send_cmd_node(uint8_t node, can_command_id_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    if (!node_valid(node)) return ESP_ERR_INVALID_ARG;
    return send_cmd_raw(CAN_CMD_ID_BASE + node, cmd, payload, payload_len);
}

static esp_err_t send_cmd_broadcast(can_command_id_t cmd, const uint8_t *payload, uint8_t payload_len)
{
    return send_cmd_raw(CAN_CMD_ID_BROADCAST, cmd, payload, payload_len);
}

static esp_err_t wait_response(uint8_t node, can_command_id_t expected_cmd, can_master_response_t *out, uint32_t timeout_ms)
{
    if (out == NULL || !node_valid(node)) return ESP_ERR_INVALID_ARG;

    TickType_t timeout = pdMS_TO_TICKS(timeout_ms);
    TickType_t start = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if ((now - start) >= timeout) return ESP_ERR_TIMEOUT;

        TickType_t remain = timeout - (now - start);
        can_master_response_t r = {0};
        if (xQueueReceive(s_resp_q, &r, remain) != pdTRUE) return ESP_ERR_TIMEOUT;

        if (r.node_id == node && r.cmd == (uint8_t)expected_cmd) {
            *out = r;
            return (r.result == CAN_PROTO_OK) ? ESP_OK : ESP_FAIL;
        }

        ESP_LOGW(TAG,
                 "Skipping unmatched response node=%u cmd=%s, waiting for node=%u cmd=%s",
                 (unsigned)r.node_id,
                 cmd_to_str(r.cmd),
                 (unsigned)node,
                 cmd_to_str((uint8_t)expected_cmd));
    }
}

static esp_err_t send_and_wait(uint8_t node, can_command_id_t cmd, const uint8_t *payload, uint8_t payload_len, can_master_response_t *out, uint32_t timeout_ms)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err = send_cmd_node(node, cmd, payload, payload_len);
    if (err != ESP_OK) return err;
    return wait_response(node, cmd, out, timeout_ms);
}

esp_err_t can_master_init(void)
{
    if (s_ready) return ESP_OK;

    if (s_resp_q == NULL) {
        s_resp_q = xQueueCreate(RESP_QUEUE_LEN, sizeof(can_master_response_t));
        if (s_resp_q == NULL) return ESP_ERR_NO_MEM;
    }

    if (s_spi_lock == NULL) {
        s_spi_lock = xSemaphoreCreateMutex();
        if (s_spi_lock == NULL) return ESP_ERR_NO_MEM;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CAN_MASTER_SPI_MOSI_GPIO,
        .miso_io_num = CAN_MASTER_SPI_MISO_GPIO,
        .sclk_io_num = CAN_MASTER_SPI_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    esp_err_t err = spi_bus_initialize(CAN_MASTER_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = CAN_MASTER_SPI_CLOCK_HZ,
        .mode = 0,
        .spics_io_num = CAN_MASTER_SPI_CS_GPIO,
        .queue_size = 4,
        .flags = 0,
    };

    if (s_spi == NULL) {
        err = spi_bus_add_device(CAN_MASTER_SPI_HOST, &dev_cfg, &s_spi);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
            return err;
        }
    }

    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << CAN_MASTER_INT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    err = gpio_config(&int_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "INT gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    if (!lock_spi()) return ESP_ERR_TIMEOUT;

    err = mcp_reset_locked();
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(10));
        // First sanity read after reset to verify SPI communication.
        uint8_t canstat = 0xFF;
        if (mcp_read_reg_locked(REG_CANSTAT, &canstat) == ESP_OK) {
            ESP_LOGI(TAG, "CANSTAT after reset: 0x%02X", canstat);
        }
        err = mcp_set_mode_locked(CANCTRL_REQOP_CONFIG);
    }
    if (err == ESP_OK) err = mcp_configure_locked();
    if (err == ESP_OK) err = mcp_set_mode_locked(CANCTRL_REQOP_NORMAL);
    unlock_spi();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MCP2515 init failed: %s", esp_err_to_name(err));
        return err;
    }

    if (s_rx_task == NULL) {
        if (xTaskCreate(rx_task, "can_rx_master", RX_TASK_STACK, NULL, RX_TASK_PRIO, &s_rx_task) != pdPASS) {
            ESP_LOGE(TAG, "Unable to create RX task");
            return ESP_FAIL;
        }
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_isr_handler_add(CAN_MASTER_INT_GPIO, can_int_isr, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed: %s", esp_err_to_name(err));
        return err;
    }
    err = gpio_intr_enable(CAN_MASTER_INT_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_intr_enable failed: %s", esp_err_to_name(err));
        return err;
    }

    s_ready = true;
    ESP_LOGI(TAG,
             "CAN MASTER (MCP2515) ready host=%d sck=%d mosi=%d miso=%d cs=%d int=%d osc=%" PRIu32 " bitrate=%" PRIu32,
             (int)CAN_MASTER_SPI_HOST,
             (int)CAN_MASTER_SPI_SCK_GPIO,
             (int)CAN_MASTER_SPI_MOSI_GPIO,
             (int)CAN_MASTER_SPI_MISO_GPIO,
             (int)CAN_MASTER_SPI_CS_GPIO,
             (int)CAN_MASTER_INT_GPIO,
             (uint32_t)CAN_MASTER_MCP2515_OSC_HZ,
             (uint32_t)CAN_MASTER_BITRATE);
    return ESP_OK;
}

bool can_master_is_ready(void) { return s_ready; }

void can_master_clear_pending_responses(void)
{
    if (s_resp_q == NULL) return;

    can_master_response_t discarded = {0};
    while (xQueueReceive(s_resp_q, &discarded, 0) == pdTRUE) {
    }
}

esp_err_t can_master_arm(uint8_t node_id, can_master_response_t *out, uint32_t timeout_ms)
{
    return send_and_wait(node_id, CAN_CMD_ARM, NULL, 0, out, timeout_ms);
}

esp_err_t can_master_disarm(uint8_t node_id, can_master_response_t *out, uint32_t timeout_ms)
{
    return send_and_wait(node_id, CAN_CMD_DISARM, NULL, 0, out, timeout_ms);
}

esp_err_t can_master_arm_all(void) { return send_cmd_broadcast(CAN_CMD_ARM, NULL, 0); }
esp_err_t can_master_disarm_all(void) { return send_cmd_broadcast(CAN_CMD_DISARM, NULL, 0); }

esp_err_t can_master_home(uint8_t node_id, can_master_response_t *out, uint32_t timeout_ms)
{
    return send_and_wait(node_id, CAN_CMD_HOME, NULL, 0, out, timeout_ms);
}

esp_err_t can_master_home_broadcast_all(void)
{
    return send_cmd_broadcast(CAN_CMD_HOME, NULL, 0);
}

esp_err_t can_master_prepare_slot(uint8_t node_id, uint8_t slot, can_master_response_t *out, uint32_t timeout_ms)
{
    uint8_t p[1] = {slot};
    return send_and_wait(node_id, CAN_CMD_PREPARE, p, sizeof(p), out, timeout_ms);
}

esp_err_t can_master_program_run(uint8_t node_id, uint8_t slot, can_master_response_t *out, uint32_t timeout_ms)
{
    uint8_t p[1] = {slot};
    return send_and_wait(node_id, CAN_CMD_PROGRAM_RUN, p, sizeof(p), out, timeout_ms);
}

esp_err_t can_master_program_delete(uint8_t node_id, uint8_t slot, can_master_response_t *out, uint32_t timeout_ms)
{
    uint8_t p[1] = {slot};
    return send_and_wait(node_id, CAN_CMD_PROGRAM_DELETE, p, sizeof(p), out, timeout_ms);
}

esp_err_t can_master_request_status(uint8_t node_id, can_master_response_t *out, uint32_t timeout_ms)
{
    clear_info_cache(node_id);
    return send_and_wait(node_id, CAN_CMD_GET_STATUS, NULL, 0, out, timeout_ms);
}

esp_err_t can_master_stop_all(void) { return send_cmd_broadcast(CAN_CMD_STOP, NULL, 0); }
esp_err_t can_master_sync_start_all(void) { return send_cmd_broadcast(CAN_CMD_SYNC_START, NULL, 0); }

esp_err_t can_master_upload_gcode_program(uint8_t node_id, uint8_t slot, const uint8_t *program_bytes, size_t size_bytes, uint32_t timeout_ms)
{
    if (program_bytes == NULL || size_bytes == 0 || size_bytes > 0xFFFFU) return ESP_ERR_INVALID_ARG;
    if (slot >= CAN_PROGRAM_SLOT_COUNT) return ESP_ERR_INVALID_ARG;

    can_master_response_t resp = {0};
    uint8_t begin[3] = { slot, (uint8_t)(size_bytes & 0xFFU), (uint8_t)((size_bytes >> 8U) & 0xFFU) };

    esp_err_t err = send_and_wait(node_id, CAN_CMD_UPLOAD_BEGIN, begin, sizeof(begin), &resp, timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UPLOAD_BEGIN failed node=%u slot=%u result=%s", (unsigned)node_id, (unsigned)slot, can_master_protocol_result_to_str(resp.result));
        return err;
    }

    uint16_t seq = 0;
    size_t off = 0;
    while (off < size_bytes) {
        size_t chunk = size_bytes - off;
        if (chunk > 5U) chunk = 5U;

        uint8_t payload[7] = {0};
        payload[0] = (uint8_t)(seq & 0xFFU);
        payload[1] = (uint8_t)((seq >> 8U) & 0xFFU);
        memcpy(&payload[2], &program_bytes[off], chunk);

        err = send_and_wait(node_id, CAN_CMD_UPLOAD_DATA, payload, (uint8_t)(2U + chunk), &resp, timeout_ms);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UPLOAD_DATA failed node=%u seq=%u result=%s", (unsigned)node_id, (unsigned)seq, can_master_protocol_result_to_str(resp.result));
            return err;
        }

        off += chunk;
        seq++;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    err = send_and_wait(node_id, CAN_CMD_UPLOAD_END, NULL, 0, &resp, timeout_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UPLOAD_END failed node=%u result=%s", (unsigned)node_id, can_master_protocol_result_to_str(resp.result));
        return err;
    }

    ESP_LOGI(TAG, "UPLOAD done node=%u slot=%u bytes=%u", (unsigned)node_id, (unsigned)slot, (unsigned)size_bytes);
    vTaskDelay(pdMS_TO_TICKS(120));
    can_master_clear_pending_responses();
    return ESP_OK;
}

esp_err_t can_master_get_last_status(uint8_t node_id, can_master_status_t *out)
{
    if (out == NULL || !node_valid(node_id)) return ESP_ERR_INVALID_ARG;

    esp_err_t err = ESP_ERR_NOT_FOUND;
    portENTER_CRITICAL(&s_status_mux);
    if (s_status[node_id].valid) {
        *out = s_status[node_id].status;
        err = ESP_OK;
    }
    portEXIT_CRITICAL(&s_status_mux);
    return err;
}

esp_err_t can_master_wait_work_offset(uint8_t node_id, can_master_work_offset_t *out_work_offset, uint32_t timeout_ms)
{
    if (out_work_offset == NULL || !node_valid(node_id)) return ESP_ERR_INVALID_ARG;

    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    const TickType_t start = xTaskGetTickCount();

    for (;;) {
        TickType_t current = xTaskGetTickCount();
        if ((current - start) >= timeout_ticks) return ESP_ERR_TIMEOUT;

        portENTER_CRITICAL(&s_info_mux);
        if (s_info[node_id].valid && ((s_info[node_id].frame_mask & INFO_MASK_WORK_OFFSET) != 0U)) {
            *out_work_offset = s_info[node_id].work_offset;
            out_work_offset->age_ms = ticks_to_ms(current - s_info[node_id].last_seen_tick);
            portEXIT_CRITICAL(&s_info_mux);
            return ESP_OK;
        }
        portEXIT_CRITICAL(&s_info_mux);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

esp_err_t can_master_wait_sensor_values(uint8_t node_id, can_master_sensor_values_t *out_sensor_values, uint32_t timeout_ms)
{
    if (out_sensor_values == NULL || !node_valid(node_id)) return ESP_ERR_INVALID_ARG;

    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    const TickType_t start = xTaskGetTickCount();

    for (;;) {
        TickType_t now = xTaskGetTickCount();
        if ((now - start) >= timeout_ticks) return ESP_ERR_TIMEOUT;

        portENTER_CRITICAL(&s_info_mux);
        if (s_info[node_id].valid && ((s_info[node_id].frame_mask & INFO_MASK_SENSORS) == INFO_MASK_SENSORS)) {
            *out_sensor_values = s_info[node_id].sensors;
            out_sensor_values->age_ms = ticks_to_ms(now - s_info[node_id].last_seen_tick);
            portEXIT_CRITICAL(&s_info_mux);
            return ESP_OK;
        }
        portEXIT_CRITICAL(&s_info_mux);

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

bool can_master_node_is_online(uint8_t node_id, uint32_t max_age_ms)
{
    if (!node_valid(node_id)) return false;

    const TickType_t now = xTaskGetTickCount();
    const TickType_t max_age_ticks = pdMS_TO_TICKS(max_age_ms);
    bool online = false;

    portENTER_CRITICAL(&s_status_mux);
    if (s_status[node_id].valid) {
        online = (now - s_status[node_id].last_seen_tick) <= max_age_ticks;
    }
    portEXIT_CRITICAL(&s_status_mux);

    return online;
}

size_t can_master_count_online_nodes(const uint8_t *node_ids, size_t node_count, uint32_t max_age_ms)
{
    if (node_ids == NULL || node_count == 0) return 0;

    size_t online_count = 0;
    for (size_t i = 0; i < node_count; i++) {
        if (can_master_node_is_online(node_ids[i], max_age_ms)) {
            online_count++;
        }
    }
    return online_count;
}
