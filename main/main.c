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
static const uint8_t ROBOT_NODE_IDS[] = {CAN_MASTER_NODE1_ID, CAN_MASTER_NODE2_ID};
#define ROBOT_NODE_COUNT (sizeof(ROBOT_NODE_IDS) / sizeof(ROBOT_NODE_IDS[0]))

static esp_err_t init_nvs(void);
static esp_err_t init_spiffs(void);

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
