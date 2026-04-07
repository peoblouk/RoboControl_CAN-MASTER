#include <string.h>

#include "can_master.h"
#include "cmd_control.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "status_led.h"

static const char *TAG = "main_node";

// Set IDs of all robot slave nodes you want to synchronize.
static const uint8_t ROBOT_NODE_IDS[] = {CAN_MASTER_NODE1_ID, CAN_MASTER_NODE2_ID};
#define ROBOT_NODE_COUNT (sizeof(ROBOT_NODE_IDS) / sizeof(ROBOT_NODE_IDS[0]))
static const uint8_t PROGRAM_SLOT = 0;

static esp_err_t init_nvs(void);
static esp_err_t init_spiffs(void);
static void log_response(const can_master_response_t *response);
static esp_err_t run_sync_sequence(void) __attribute__((unused));

static const char GCODE_PROGRAM[] =
    "G90\n"
    "G0 X130 Y0 Z80\n"
    "G1 X145 Y0 Z70 F1200\n"
    "G1 X145 Y35 Z70 F1200\n"
    "G1 X130 Y35 Z80 F1200\n"
    "M30\n";

static void log_response(const can_master_response_t *response)
{
    if (response == NULL) {
        return;
    }

    ESP_LOGI(TAG,
             "node=%u cmd=0x%02X result=%s state=%u flags=0x%02X detail=[%u %u %u %u]",
             (unsigned)response->node_id,
             (unsigned)response->cmd,
             can_master_protocol_result_to_str(response->result),
             (unsigned)response->robot_state,
             (unsigned)response->state_flags,
             (unsigned)response->detail[0],
             (unsigned)response->detail[1],
             (unsigned)response->detail[2],
             (unsigned)response->detail[3]);
}

static esp_err_t run_sync_sequence(void)
{
    // Equivalent runtime flow is available in console command: "can seq [slot]".
    can_master_response_t response = {0};

    ESP_LOGI(TAG, "Step 1/4: ARM all nodes");
    for (size_t i = 0; i < ROBOT_NODE_COUNT; i++) {
        uint8_t node_id = ROBOT_NODE_IDS[i];
        esp_err_t err = can_master_arm(node_id, &response, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        log_response(&response);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ARM failed for node=%u", (unsigned)node_id);
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 2/4: Upload G-code program to all nodes");
    for (size_t i = 0; i < ROBOT_NODE_COUNT; i++) {
        uint8_t node_id = ROBOT_NODE_IDS[i];
        esp_err_t err = can_master_upload_gcode_program(node_id,
                                                        PROGRAM_SLOT,
                                                        (const uint8_t *)GCODE_PROGRAM,
                                                        strlen(GCODE_PROGRAM),
                                                        CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "G-code upload failed for node=%u", (unsigned)node_id);
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 3/4: PREPARE slot=%u on all nodes", (unsigned)PROGRAM_SLOT);
    for (size_t i = 0; i < ROBOT_NODE_COUNT; i++) {
        uint8_t node_id = ROBOT_NODE_IDS[i];
        esp_err_t err = can_master_prepare_slot(node_id, PROGRAM_SLOT, &response, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        log_response(&response);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "PREPARE failed for node=%u", (unsigned)node_id);
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 4/4: SYNC_START broadcast");
    return can_master_sync_start_all();
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format SPIFFS");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ret;
    }

    size_t total = 0;
    size_t used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to query SPIFFS info (%s)", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPIFFS mounted at %s total=%zu used=%zu", conf.base_path, total, used);
    return ESP_OK;
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting MAIN CAN node");
    esp_err_t nvs_err = init_nvs();
    if (nvs_err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(nvs_err));
        return;
    }

    esp_err_t spiffs_err = init_spiffs();
    if (spiffs_err != ESP_OK) {
        ESP_LOGW(TAG, "SPIFFS init failed, upload_file command will not work");
    }

    esp_err_t init_err = can_master_init();
    if (init_err != ESP_OK) {
        ESP_LOGE(TAG, "CAN master init failed: %s", esp_err_to_name(init_err));
    }
    vTaskDelay(pdMS_TO_TICKS(500));

    status_led_start(ROBOT_NODE_IDS, ROBOT_NODE_COUNT);
    cmd_control_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
