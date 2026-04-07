#include "status_led.h"

#include <string.h>

#include "can_master.h"
#include "driver/gpio.h"
#include "driver/rmt_common.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/rmt_types.h"

static const char *TAG = "status_led";

#define STATUS_LED_GPIO GPIO_NUM_2
#define STATUS_LED_RESOLUTION_HZ (10 * 1000 * 1000)
#define STATUS_LED_MEM_BLOCK_SYMBOLS 64
#define STATUS_LED_TASK_PERIOD_MS 200
#define STATUS_LED_MAX_NODES 8

#define STATUS_LED_T0H_TICKS 3
#define STATUS_LED_T0L_TICKS 9
#define STATUS_LED_T1H_TICKS 6
#define STATUS_LED_T1L_TICKS 6
#define STATUS_LED_RESET_TICKS 900

typedef enum {
    MASTER_LED_BOOT = 0,
    MASTER_LED_NONE_ONLINE,
    MASTER_LED_PARTIAL_ONLINE,
    MASTER_LED_ALL_ONLINE,
} master_led_state_t;

static rmt_channel_handle_t s_led_chan = NULL;
static rmt_encoder_handle_t s_copy_encoder = NULL;
static TaskHandle_t s_led_task = NULL;
static uint8_t s_node_ids[STATUS_LED_MAX_NODES] = {0};
static size_t s_node_count = 0;

static void fill_bit_symbol(bool one, rmt_symbol_word_t *symbol)
{
    symbol->level0 = 1;
    symbol->duration0 = one ? STATUS_LED_T1H_TICKS : STATUS_LED_T0H_TICKS;
    symbol->level1 = 0;
    symbol->duration1 = one ? STATUS_LED_T1L_TICKS : STATUS_LED_T0L_TICKS;
}

static size_t build_grb_symbols(uint8_t red, uint8_t green, uint8_t blue, rmt_symbol_word_t *symbols, size_t max_symbols)
{
    const uint8_t grb[3] = {green, red, blue};
    size_t symbol_index = 0;

    if (max_symbols < 25) {
        return 0;
    }

    for (size_t byte_index = 0; byte_index < sizeof(grb); byte_index++) {
        for (int bit = 7; bit >= 0; bit--) {
            fill_bit_symbol(((grb[byte_index] >> bit) & 0x01U) != 0U, &symbols[symbol_index++]);
        }
    }

    symbols[symbol_index].level0 = 0;
    symbols[symbol_index].duration0 = STATUS_LED_RESET_TICKS;
    symbols[symbol_index].level1 = 0;
    symbols[symbol_index].duration1 = 0;
    symbol_index++;

    return symbol_index;
}

static esp_err_t status_led_apply(uint8_t red, uint8_t green, uint8_t blue)
{
    if (s_led_chan == NULL || s_copy_encoder == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    rmt_symbol_word_t symbols[25] = {0};
    size_t symbol_count = build_grb_symbols(red, green, blue, symbols, sizeof(symbols) / sizeof(symbols[0]));
    if (symbol_count == 0) {
        return ESP_ERR_INVALID_SIZE;
    }

    rmt_transmit_config_t tx_cfg = {
        .loop_count = 0,
        .flags = {
            .eot_level = 0,
            .queue_nonblocking = 0,
        },
    };

    esp_err_t err = rmt_transmit(s_led_chan, s_copy_encoder, symbols, symbol_count * sizeof(symbols[0]), &tx_cfg);
    if (err != ESP_OK) {
        return err;
    }

    return rmt_tx_wait_all_done(s_led_chan, 100);
}

static master_led_state_t status_led_get_state(void)
{
    if (!can_master_is_ready()) {
        return MASTER_LED_BOOT;
    }

    size_t online_count = can_master_count_online_nodes(s_node_ids, s_node_count, CAN_MASTER_NODE_ONLINE_TIMEOUT_MS);
    if (online_count == 0) {
        return MASTER_LED_NONE_ONLINE;
    }
    if (online_count < s_node_count) {
        return MASTER_LED_PARTIAL_ONLINE;
    }
    return MASTER_LED_ALL_ONLINE;
}

static void status_led_task(void *arg)
{
    (void)arg;

    master_led_state_t prev_state = (master_led_state_t)(-1);

    for (;;) {
        master_led_state_t state = status_led_get_state();

        if (state != prev_state) {
            uint8_t red = 0;
            uint8_t green = 0;
            uint8_t blue = 0;

            if (state == MASTER_LED_BOOT) {
                blue = 16;
            } else if (state == MASTER_LED_NONE_ONLINE) {
                red = 20;
            } else if (state == MASTER_LED_PARTIAL_ONLINE) {
                red = 18;
                green = 8;
            } else if (state == MASTER_LED_ALL_ONLINE) {
                green = 18;
            }

            esp_err_t err = status_led_apply(red, green, blue);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "LED update failed: %s", esp_err_to_name(err));
            }

            prev_state = state;
        }

        vTaskDelay(pdMS_TO_TICKS(STATUS_LED_TASK_PERIOD_MS));
    }
}

void status_led_start(const uint8_t *node_ids, size_t node_count)
{
    if (s_led_task != NULL) {
        return;
    }

    if (node_ids == NULL || node_count == 0) {
        ESP_LOGW(TAG, "No configured node IDs for status LED");
        return;
    }

    s_node_count = (node_count > STATUS_LED_MAX_NODES) ? STATUS_LED_MAX_NODES : node_count;
    memcpy(s_node_ids, node_ids, s_node_count);

    rmt_tx_channel_config_t chan_cfg = {
        .gpio_num = STATUS_LED_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = STATUS_LED_RESOLUTION_HZ,
        .mem_block_symbols = STATUS_LED_MEM_BLOCK_SYMBOLS,
        .trans_queue_depth = 1,
        .intr_priority = 0,
        .flags = {
            .invert_out = 0,
            .with_dma = 0,
            .io_loop_back = 0,
            .io_od_mode = 0,
            .allow_pd = 0,
        },
    };

    esp_err_t err = rmt_new_tx_channel(&chan_cfg, &s_led_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
        return;
    }

    err = rmt_enable(s_led_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_enable failed: %s", esp_err_to_name(err));
        return;
    }

    rmt_copy_encoder_config_t encoder_cfg;
    memset(&encoder_cfg, 0, sizeof(encoder_cfg));
    err = rmt_new_copy_encoder(&encoder_cfg, &s_copy_encoder);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "rmt_new_copy_encoder failed: %s", esp_err_to_name(err));
        return;
    }

    err = status_led_apply(0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Initial LED clear failed: %s", esp_err_to_name(err));
    }

    if (xTaskCreate(status_led_task, "status_led", 3072, NULL, 1, &s_led_task) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create status LED task");
        return;
    }

    ESP_LOGI(TAG, "Status LED started on GPIO %d for %u nodes", (int)STATUS_LED_GPIO, (unsigned)s_node_count);
}
