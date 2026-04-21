#include <stdio.h>
#include <stdlib.h>
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
#define RUN_SYNC_SEQUENCE_ON_BOOT 0

#define CAN_STATE_FLAG_ARMED (1U << 0)
#define CAN_STATE_FLAG_REFERENCED (1U << 2)
#define CAN_STATE_FLAG_TCP_EST (1U << 3)
#define CAN_STATE_FLAG_PROGRAM_RUNNING (1U << 6)
#define CAN_STATE_FLAG_ERROR (1U << 7)

#define ROBOT_STATE_READY 2U
#define ROBOT_STATE_RUNNING 3U
#define ROBOT_STATE_ERROR 5U

#define CAN_SLOT_NONE 0xFFU
#define SYNC_SEQUENCE_STATUS_POLL_MS 100U
#define SYNC_SEQUENCE_READY_GRACE_MS 1200U
#define SYNC_SEQUENCE_TIMEOUT_MS 120000U

#define NODE2_FORWARD_SLOT 0U
#define NODE1_FORWARD_SLOT 0U
#define NODE1_RETURN_SLOT 1U
#define NODE2_RETURN_SLOT 1U

typedef struct {
    uint8_t node_id;
    uint8_t slot;
    const char *spiffs_path;
    const char *label;
} sync_sequence_step_t;

static esp_err_t init_nvs(void);
static esp_err_t init_spiffs(void);
static void log_response(const can_master_response_t *response);
static esp_err_t load_file_from_spiffs(const char *path, uint8_t **out_data, size_t *out_size);
static esp_err_t request_and_read_status(uint8_t node_id, can_master_status_t *out_status);
static bool status_is_ready_for_program(const can_master_status_t *status);
static bool status_is_running_expected_slot(const can_master_status_t *status, uint8_t slot);
static esp_err_t wait_for_node_ready(uint8_t node_id, uint8_t expected_slot, uint32_t timeout_ms);
static esp_err_t upload_step_program(const sync_sequence_step_t *step);
static esp_err_t run_step_program(const sync_sequence_step_t *step);
static esp_err_t run_sync_sequence(void) __attribute__((unused));

static const sync_sequence_step_t SYNC_SEQUENCE_STEPS[] = {
    { CAN_MASTER_NODE2_ID, NODE2_FORWARD_SLOT, "/spiffs/node2_slot0.gcode", "node2 slot0: HOME -> pick A -> place B -> HOME" },
    { CAN_MASTER_NODE1_ID, NODE1_FORWARD_SLOT, "/spiffs/node1_slot0.gcode", "node1 slot0: HOME -> pick B -> place C -> HOME" },
    { CAN_MASTER_NODE1_ID, NODE1_RETURN_SLOT, "/spiffs/node1_slot1.gcode", "node1 slot1: HOME -> pick C -> place B -> HOME" },
    { CAN_MASTER_NODE2_ID, NODE2_RETURN_SLOT, "/spiffs/node2_slot1.gcode", "node2 slot1: HOME -> pick B -> place A -> HOME" },
};

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

static esp_err_t load_file_from_spiffs(const char *path, uint8_t **out_data, size_t *out_size)
{
    if (path == NULL || out_data == NULL || out_size == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        return ESP_ERR_NOT_FOUND;
    }

    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return ESP_FAIL;
    }

    long file_size = ftell(f);
    if (file_size <= 0 || file_size > 0xFFFFL) {
        fclose(f);
        return (file_size <= 0) ? ESP_ERR_INVALID_SIZE : ESP_ERR_INVALID_ARG;
    }

    rewind(f);

    uint8_t *data = (uint8_t *)malloc((size_t)file_size);
    if (data == NULL) {
        fclose(f);
        return ESP_ERR_NO_MEM;
    }

    size_t read_count = fread(data, 1, (size_t)file_size, f);
    fclose(f);
    if (read_count != (size_t)file_size) {
        free(data);
        return ESP_FAIL;
    }

    *out_data = data;
    *out_size = (size_t)file_size;
    return ESP_OK;
}

static esp_err_t request_and_read_status(uint8_t node_id, can_master_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_master_clear_pending_responses();

    can_master_response_t resp = {0};
    esp_err_t err = can_master_request_status(node_id, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err != ESP_OK) {
        return err;
    }

    for (int attempt = 0; attempt < 10; attempt++) {
        err = can_master_get_last_status(node_id, out_status);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    return err;
}

static bool status_is_ready_for_program(const can_master_status_t *status)
{
    if (status == NULL) return false;

    return status->robot_state == ROBOT_STATE_READY &&
           status->active_slot == CAN_SLOT_NONE &&
           (status->state_flags & CAN_STATE_FLAG_ARMED) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_REFERENCED) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_TCP_EST) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_PROGRAM_RUNNING) == 0U &&
           (status->state_flags & CAN_STATE_FLAG_ERROR) == 0U;
}

static bool status_is_running_expected_slot(const can_master_status_t *status, uint8_t slot)
{
    if (status == NULL) return false;

    return status->robot_state == ROBOT_STATE_RUNNING ||
           (status->state_flags & CAN_STATE_FLAG_PROGRAM_RUNNING) != 0U ||
           status->active_slot == slot;
}

static esp_err_t wait_for_node_ready(uint8_t node_id, uint8_t expected_slot, uint32_t timeout_ms)
{
    const TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool saw_running = false;
    esp_err_t last_err = ESP_OK;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - start_tick) >= timeout_ticks) {
            return (last_err != ESP_OK) ? last_err : ESP_ERR_TIMEOUT;
        }

        can_master_status_t status = {0};
        last_err = request_and_read_status(node_id, &status);
        if (last_err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(SYNC_SEQUENCE_STATUS_POLL_MS));
            continue;
        }

        if (status.robot_state == ROBOT_STATE_ERROR || (status.state_flags & CAN_STATE_FLAG_ERROR) != 0U) {
            ESP_LOGE(TAG,
                     "Node %u entered error state=%u flags=0x%02X prepared=%u active=%u proto=%s robot_err=%u",
                     (unsigned)status.node_id,
                     (unsigned)status.robot_state,
                     (unsigned)status.state_flags,
                     (unsigned)status.prepared_slot,
                     (unsigned)status.active_slot,
                     can_master_protocol_result_to_str(status.last_protocol_error),
                     (unsigned)status.last_robot_error);
            return ESP_FAIL;
        }

        if (expected_slot != CAN_SLOT_NONE && status_is_running_expected_slot(&status, expected_slot)) {
            saw_running = true;
        }

        const uint32_t elapsed_ms = (uint32_t)pdTICKS_TO_MS(now - start_tick);
        if (status_is_ready_for_program(&status) &&
            (saw_running || elapsed_ms >= SYNC_SEQUENCE_READY_GRACE_MS)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(SYNC_SEQUENCE_STATUS_POLL_MS));
    }
}

static esp_err_t upload_step_program(const sync_sequence_step_t *step)
{
    if (step == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t *file_data = NULL;
    size_t file_size = 0;
    esp_err_t err = load_file_from_spiffs(step->spiffs_path, &file_data, &file_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read %s (%s)", step->spiffs_path, esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG,
             "Upload %s -> node=%u slot=%u bytes=%u",
             step->spiffs_path,
             (unsigned)step->node_id,
             (unsigned)step->slot,
             (unsigned)file_size);

    err = can_master_upload_gcode_program(step->node_id,
                                          step->slot,
                                          file_data,
                                          file_size,
                                          CAN_MASTER_RESPONSE_TIMEOUT_MS);
    free(file_data);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload failed for %s (%s)", step->label, esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static esp_err_t run_step_program(const sync_sequence_step_t *step)
{
    if (step == NULL) return ESP_ERR_INVALID_ARG;

    can_master_response_t response = {0};
    ESP_LOGI(TAG, "Run %s", step->label);
    esp_err_t err = can_master_program_run(step->node_id, step->slot, &response, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    log_response(&response);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "PROGRAM_RUN failed for node=%u slot=%u (%s)",
                 (unsigned)step->node_id,
                 (unsigned)step->slot,
                 esp_err_to_name(err));
        return err;
    }

    err = wait_for_node_ready(step->node_id, step->slot, SYNC_SEQUENCE_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,
                 "Timed out waiting for node=%u slot=%u to finish (%s)",
                 (unsigned)step->node_id,
                 (unsigned)step->slot,
                 esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Finished %s", step->label);
    return ESP_OK;
}

static esp_err_t run_sync_sequence(void)
{
    // Equivalent runtime flow is available in console command: "can relay [cycles]".
    can_master_response_t response = {0};

    ESP_LOGI(TAG, "Step 1/4: ARM both nodes");
    for (size_t i = 0; i < ROBOT_NODE_COUNT; i++) {
        uint8_t node_id = ROBOT_NODE_IDS[i];
        esp_err_t err = can_master_arm(node_id, &response, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        log_response(&response);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ARM failed for node=%u", (unsigned)node_id);
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 2/4: Upload relay G-code files from SPIFFS");
    for (size_t i = 0; i < sizeof(SYNC_SEQUENCE_STEPS) / sizeof(SYNC_SEQUENCE_STEPS[0]); i++) {
        esp_err_t err = upload_step_program(&SYNC_SEQUENCE_STEPS[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 3/4: Broadcast HOME to both nodes and wait for READY");
    esp_err_t err = can_master_home_broadcast_all();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Broadcast HOME failed: %s", esp_err_to_name(err));
        return err;
    }
    for (size_t i = 0; i < ROBOT_NODE_COUNT; i++) {
        uint8_t node_id = ROBOT_NODE_IDS[i];
        err = wait_for_node_ready(node_id, CAN_SLOT_NONE, SYNC_SEQUENCE_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "HOME/READY wait failed for node=%u", (unsigned)node_id);
            return err;
        }
    }

    ESP_LOGI(TAG, "Step 4/4: Run one full relay cycle");
    for (size_t i = 0; i < sizeof(SYNC_SEQUENCE_STEPS) / sizeof(SYNC_SEQUENCE_STEPS[0]); i++) {
        err = run_step_program(&SYNC_SEQUENCE_STEPS[i]);
        if (err != ESP_OK) {
            return err;
        }
    }

    ESP_LOGI(TAG, "Relay cycle completed");
    return ESP_OK;
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

#if RUN_SYNC_SEQUENCE_ON_BOOT
    esp_err_t seq_err = run_sync_sequence();
    if (seq_err != ESP_OK) {
        ESP_LOGE(TAG, "run_sync_sequence failed: %s", esp_err_to_name(seq_err));
    }
#endif

    status_led_start(ROBOT_NODE_IDS, ROBOT_NODE_COUNT);
    cmd_control_start();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
