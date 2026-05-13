#include "cmd_control.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_master.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "linenoise/linenoise.h"
#include "sdkconfig.h"

static const char *TAG = "cmd";
static esp_console_repl_t *s_repl = NULL;
static const uint8_t CONFIGURED_NODE_IDS[] = {CAN_MASTER_NODE1_ID, CAN_MASTER_NODE2_ID};
#define CONFIGURED_NODE_COUNT (sizeof(CONFIGURED_NODE_IDS) / sizeof(CONFIGURED_NODE_IDS[0]))
#define CAN_STATE_FLAG_ARMED (1U << 0)
#define CAN_STATE_FLAG_REFERENCED (1U << 2)
#define CAN_STATE_FLAG_TCP_EST (1U << 3)
#define CAN_STATE_FLAG_PROGRAM_RUNNING (1U << 6)
#define CAN_STATE_FLAG_ERROR (1U << 7)

#define ROBOT_STATE_DISARMED 0U
#define ROBOT_STATE_UNREFERENCED 1U
#define ROBOT_STATE_READY 2U
#define ROBOT_STATE_RUNNING 3U
#define ROBOT_STATE_READY_FOR_SYNC 4U
#define ROBOT_STATE_ERROR 5U

#define CAN_SLOT_NONE 0xFFU

#define RELAY_STATUS_POLL_MS 100U
#define RELAY_READY_GRACE_MS 1200U
#define RELAY_READY_TIMEOUT_MS 120000U
#define RELAY_HOME_SETTLE_TIMEOUT_MS 120000U
#define RELAY_TASK_STACK_SIZE 6144U
#define RELAY_TASK_PRIORITY 5U

#define RELAY_NODE1_FORWARD_SLOT 0U
#define RELAY_NODE2_FORWARD_SLOT 0U
#define RELAY_NODE2_RETURN_SLOT 1U
#define RELAY_NODE1_RETURN_SLOT 1U

#define SYNC_MEASURE_DEFAULT_DURATION_MS 2U
#define SYNC_MEASURE_MIN_DURATION_MS 1U
#define SYNC_MEASURE_MAX_DURATION_MS 60000U
#define SYNC_MEASURE_SLOT 3U
#define SYNC_MEASURE_STOP_MARGIN_MS 3000U
#define SYNC_MEASURE_GCODE_BUFFER_SIZE 192U
#define RUN_SYNC_DEFAULT_SLOT 0U

static const char DEFAULT_GCODE[] =
    "G21\n"
    "G90\n"
    "F700\n"
    "\n"
    "G0 X0 Y0 Z25 P-20\n"
    "G4 P200\n"
    "G1 X30 Y0 Z25 P-20\n"
    "G4 P200\n"
    "G1 X30 Y30 Z25 P-20\n"
    "G4 P200\n"
    "G1 X0 Y30 Z25 P-20\n"
    "G4 P200\n"
    "G1 X0 Y0 Z25 P-20\n"
    "G4 P200\n"
    "G1 X0 Y0 Z0 P-53\n"
    "M30\n";

typedef struct {
    uint8_t node_id;
    uint8_t slot;
    const char *label;
} relay_step_t;

static volatile bool s_relay_running = false;
static volatile bool s_relay_stop_requested = false;
static volatile uint32_t s_relay_target_cycles = 0;
static volatile uint32_t s_relay_active_cycle = 0;
static bool s_sync_measure_close_next = true;
static portMUX_TYPE s_relay_mux = portMUX_INITIALIZER_UNLOCKED;

static void print_response_line(const can_master_response_t *resp);
static bool relay_is_running(void);
static bool relay_stop_requested(void);
static void relay_request_stop(void);
static void relay_set_cycle_progress(uint32_t active_cycle);
static void relay_get_progress(uint32_t *out_active_cycle, uint32_t *out_target_cycles);
static esp_err_t relay_execute_cycles(uint32_t cycles);
static void relay_task(void *arg);

static bool parse_slot_arg(const char *arg, uint8_t *out_slot)
{
    if (arg == NULL || out_slot == NULL) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed_slot = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || parsed_slot >= CAN_PROGRAM_SLOT_COUNT) {
        return false;
    }

    *out_slot = (uint8_t)parsed_slot;
    return true;
}

static bool parse_cycle_count_arg(const char *arg, uint32_t *out_cycles)
{
    if (arg == NULL || out_cycles == NULL) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed_cycles = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || parsed_cycles == 0UL || parsed_cycles > 100000UL) {
        return false;
    }

    *out_cycles = (uint32_t)parsed_cycles;
    return true;
}

static bool normalize_spiffs_path(const char *input, char *output, size_t output_size)
{
    if (input == NULL || output == NULL || output_size == 0U) {
        return false;
    }

    int written = 0;
    if (strncmp(input, "/spiffs/", 8) == 0) {
        written = snprintf(output, output_size, "%s", input);
    } else if (input[0] == '/') {
        written = snprintf(output, output_size, "/spiffs%s", input);
    } else {
        written = snprintf(output, output_size, "/spiffs/%s", input);
    }
    return written > 0 && (size_t)written < output_size;
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

static bool parse_node_or_all(const char *arg, uint8_t *out_node, bool *out_all)
{
    if (arg == NULL || out_node == NULL || out_all == NULL) {
        return false;
    }

    if (strcmp(arg, "all") == 0) {
        *out_all = true;
        *out_node = 0;
        return true;
    }

    char *end = NULL;
    unsigned long parsed = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || parsed == 0UL || parsed > CAN_NODE_ID_MAX) {
        return false;
    }

    *out_all = false;
    *out_node = (uint8_t)parsed;
    return true;
}

static const char *robot_state_to_str(uint8_t state)
{
    switch (state) {
        case ROBOT_STATE_DISARMED: return "DISARMED";
        case ROBOT_STATE_UNREFERENCED: return "UNREFERENCED";
        case ROBOT_STATE_READY: return "READY";
        case ROBOT_STATE_RUNNING: return "RUNNING";
        case ROBOT_STATE_READY_FOR_SYNC: return "READY_FOR_SYNC";
        case ROBOT_STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

static void print_status_snapshot(const can_master_status_t *status)
{
    if (status == NULL) {
        return;
    }

    printf("node=%u state=%s(%u) flags=0x%02X prepared_slot=%u active_slot=%u proto_err=%s robot_err=%u\n",
           (unsigned)status->node_id,
           robot_state_to_str(status->robot_state),
           (unsigned)status->robot_state,
           (unsigned)status->state_flags,
           (unsigned)status->prepared_slot,
           (unsigned)status->active_slot,
           can_master_protocol_result_to_str(status->last_protocol_error),
           (unsigned)status->last_robot_error);
}

static bool relay_is_running(void)
{
    bool running = false;
    taskENTER_CRITICAL(&s_relay_mux);
    running = s_relay_running;
    taskEXIT_CRITICAL(&s_relay_mux);
    return running;
}

static bool relay_stop_requested(void)
{
    bool stop_requested = false;
    taskENTER_CRITICAL(&s_relay_mux);
    stop_requested = s_relay_stop_requested;
    taskEXIT_CRITICAL(&s_relay_mux);
    return stop_requested;
}

static void relay_request_stop(void)
{
    taskENTER_CRITICAL(&s_relay_mux);
    s_relay_stop_requested = true;
    taskEXIT_CRITICAL(&s_relay_mux);
}

static void relay_set_cycle_progress(uint32_t active_cycle)
{
    taskENTER_CRITICAL(&s_relay_mux);
    s_relay_active_cycle = active_cycle;
    taskEXIT_CRITICAL(&s_relay_mux);
}

static void relay_get_progress(uint32_t *out_active_cycle, uint32_t *out_target_cycles)
{
    taskENTER_CRITICAL(&s_relay_mux);
    if (out_active_cycle != NULL) {
        *out_active_cycle = s_relay_active_cycle;
    }
    if (out_target_cycles != NULL) {
        *out_target_cycles = s_relay_target_cycles;
    }
    taskEXIT_CRITICAL(&s_relay_mux);
}

static esp_err_t read_cached_status(uint8_t node_id, can_master_status_t *out_status)
{
    if (out_status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!can_master_node_is_online(node_id, CAN_MASTER_NODE_ONLINE_TIMEOUT_MS)) {
        return ESP_ERR_NOT_FOUND;
    }

    return can_master_get_last_status(node_id, out_status);
}

static bool status_is_ready_for_program(const can_master_status_t *status)
{
    if (status == NULL) {
        return false;
    }

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
    if (status == NULL) {
        return false;
    }

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

        if (relay_stop_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        can_master_status_t status = {0};
        last_err = read_cached_status(node_id, &status);
        if (last_err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(RELAY_STATUS_POLL_MS));
            continue;
        }

        if (status.robot_state == ROBOT_STATE_ERROR || (status.state_flags & CAN_STATE_FLAG_ERROR) != 0U) {
            print_status_snapshot(&status);
            return ESP_FAIL;
        }

        if (expected_slot != CAN_SLOT_NONE && status_is_running_expected_slot(&status, expected_slot)) {
            saw_running = true;
        }

        const uint32_t elapsed_ms = (uint32_t)pdTICKS_TO_MS(now - start_tick);
        if (status_is_ready_for_program(&status) &&
            (saw_running || elapsed_ms >= RELAY_READY_GRACE_MS)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(RELAY_STATUS_POLL_MS));
    }
}

static esp_err_t relay_arm_and_home_node(uint8_t node_id)
{
    can_master_response_t resp = {0};

    if (relay_stop_requested()) {
        return ESP_ERR_INVALID_STATE;
    }

    can_master_clear_pending_responses();
    esp_err_t err = can_master_arm(node_id, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err != ESP_OK) {
        if (resp.node_id != 0U) print_response_line(&resp);
        printf("ERR: node=%u arm failed (%s)\n", (unsigned)node_id, esp_err_to_name(err));
        return err;
    }
    print_response_line(&resp);

    if (relay_stop_requested()) {
        return ESP_ERR_INVALID_STATE;
    }

    can_master_clear_pending_responses();
    err = can_master_home(node_id, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err != ESP_OK) {
        if (resp.node_id != 0U) print_response_line(&resp);
        printf("ERR: node=%u home failed (%s)\n", (unsigned)node_id, esp_err_to_name(err));
        return err;
    }
    print_response_line(&resp);

    err = wait_for_node_ready(node_id, CAN_SLOT_NONE, RELAY_HOME_SETTLE_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: node=%u did not become READY after home (%s)\n", (unsigned)node_id, esp_err_to_name(err));
        return err;
    }

    printf("OK: node=%u referenced and ready\n", (unsigned)node_id);
    return ESP_OK;
}

static esp_err_t relay_broadcast_home_all(void)
{
    if (relay_stop_requested()) {
        return ESP_ERR_INVALID_STATE;
    }

    can_master_clear_pending_responses();
    printf("RELAY: broadcast HOME to both nodes\n");
    esp_err_t err = can_master_home_broadcast_all();
    if (err != ESP_OK) {
        printf("ERR: broadcast HOME failed (%s)\n", esp_err_to_name(err));
        return err;
    }

    err = wait_for_node_ready(CAN_MASTER_NODE1_ID, CAN_SLOT_NONE, RELAY_HOME_SETTLE_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: node=%u broadcast HOME did not finish cleanly (%s)\n",
               (unsigned)CAN_MASTER_NODE1_ID,
               esp_err_to_name(err));
        return err;
    }

    err = wait_for_node_ready(CAN_MASTER_NODE2_ID, CAN_SLOT_NONE, RELAY_HOME_SETTLE_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: node=%u broadcast HOME did not finish cleanly (%s)\n",
               (unsigned)CAN_MASTER_NODE2_ID,
               esp_err_to_name(err));
        return err;
    }

    printf("OK: synchronized HOME reference finished on both nodes\n");
    return ESP_OK;
}

static esp_err_t relay_run_step(const relay_step_t *step)
{
    if (step == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_master_response_t resp = {0};
    printf("RELAY: run node=%u slot=%u (%s)\n",
           (unsigned)step->node_id,
           (unsigned)step->slot,
           step->label);

    if (relay_stop_requested()) {
        return ESP_ERR_INVALID_STATE;
    }

    can_master_clear_pending_responses();
    esp_err_t err = can_master_program_run(step->node_id, step->slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err != ESP_OK) {
        if (resp.node_id != 0U) print_response_line(&resp);
        printf("ERR: node=%u slot=%u run failed (%s)\n",
               (unsigned)step->node_id,
               (unsigned)step->slot,
               esp_err_to_name(err));
        return err;
    }
    print_response_line(&resp);

    err = wait_for_node_ready(step->node_id, step->slot, RELAY_READY_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: node=%u slot=%u did not finish cleanly (%s)\n",
               (unsigned)step->node_id,
               (unsigned)step->slot,
               esp_err_to_name(err));
        return err;
    }

    printf("OK: node=%u slot=%u finished (%s)\n",
           (unsigned)step->node_id,
           (unsigned)step->slot,
           step->label);
    return ESP_OK;
}

static void print_configured_nodes(void)
{
    printf("Configured nodes (online timeout %u ms):\n", (unsigned)CAN_MASTER_NODE_ONLINE_TIMEOUT_MS);
    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t node_id = CONFIGURED_NODE_IDS[i];
        printf("  node=%u %s\n",
               (unsigned)node_id,
               can_master_node_is_online(node_id, CAN_MASTER_NODE_ONLINE_TIMEOUT_MS) ? "online" : "offline");
    }
}

static void print_response_line(const can_master_response_t *resp)
{
    if (resp == NULL) {
        return;
    }

    printf("node=%u cmd=0x%02X result=%s state=%u flags=0x%02X detail=[%u %u %u %u]\n",
           (unsigned)resp->node_id,
           (unsigned)resp->cmd,
           can_master_protocol_result_to_str(resp->result),
           (unsigned)resp->robot_state,
           (unsigned)resp->state_flags,
           (unsigned)resp->detail[0],
           (unsigned)resp->detail[1],
           (unsigned)resp->detail[2],
           (unsigned)resp->detail[3]);
}

static void print_fixed_0p1_i16(int16_t value, int16_t invalid_value)
{
    if (value == invalid_value) {
        printf("n/a");
        return;
    }

    int32_t signed_value = (int32_t)value;
    if (signed_value < 0) {
        printf("-");
        signed_value = -signed_value;
    }
    printf("%" PRId32 ".%" PRId32, signed_value / 10, signed_value % 10);
}

static void print_work_offset_values(const int16_t values_0p1mm[3])
{
    printf("[");
    for (size_t i = 0; i < 3U; i++) {
        if (i > 0) printf(" ");
        print_fixed_0p1_i16(values_0p1mm[i], CAN_INFO_VALUE_INVALID_I16);
    }
    printf("]");
}

static void print_sensor_values_line(const can_master_sensor_values_t *sensors)
{
    if (sensors == NULL) return;

    printf("node=%u sensors_deg=[", (unsigned)sensors->node_id);
    for (size_t i = 0; i < 6U; i++) {
        if (i > 0) printf(" ");
        print_fixed_0p1_i16(sensors->values_0p1deg[i], CAN_INFO_VALUE_INVALID_I16);
    }
    printf("]\n");
}

static int cmd_can_nodes(void)
{
    print_configured_nodes();
    return 0;
}

static int cmd_can_sensors_like(const char *target)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            can_master_response_t resp = {0};
            esp_err_t err = can_master_request_status(CONFIGURED_NODE_IDS[i], &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err != ESP_OK) {
                printf("ERR: node=%u sensors failed (%s)\n", (unsigned)CONFIGURED_NODE_IDS[i], esp_err_to_name(err));
                continue;
            }

            can_master_sensor_values_t sensors = {0};
            err = can_master_wait_sensor_values(CONFIGURED_NODE_IDS[i], &sensors, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err == ESP_OK) {
                print_sensor_values_line(&sensors);
            } else {
                printf("ERR: node=%u sensors failed (%s)\n", (unsigned)CONFIGURED_NODE_IDS[i], esp_err_to_name(err));
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = can_master_request_status(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: sensors failed (%s)\n", esp_err_to_name(err));
        return 0;
    }

    can_master_sensor_values_t sensors = {0};
    err = can_master_wait_sensor_values(node, &sensors, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err == ESP_OK) {
        print_sensor_values_line(&sensors);
    } else {
        printf("ERR: sensors failed (%s)\n", esp_err_to_name(err));
    }
    return 0;
}

static int cmd_can_arm_like(bool do_arm, const char *target)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            can_master_response_t resp = {0};
            esp_err_t err = do_arm
                ? can_master_arm(CONFIGURED_NODE_IDS[i], &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS)
                : can_master_disarm(CONFIGURED_NODE_IDS[i], &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err == ESP_OK) {
                print_response_line(&resp);
            } else {
                printf("ERR: node=%u %s failed\n",
                       (unsigned)CONFIGURED_NODE_IDS[i],
                       do_arm ? "arm" : "disarm");
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = do_arm
        ? can_master_arm(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS)
        : can_master_disarm(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err == ESP_OK) {
        print_response_line(&resp);
    } else {
        printf("ERR: %s failed\n", do_arm ? "arm" : "disarm");
    }
    return 0;
}

static int cmd_can_home_like(const char *target)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            can_master_response_t resp = {0};
            esp_err_t err = can_master_home(CONFIGURED_NODE_IDS[i], &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err == ESP_OK) {
                print_response_line(&resp);
            } else {
                printf("ERR: node=%u home failed\n", (unsigned)CONFIGURED_NODE_IDS[i]);
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = can_master_home(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err == ESP_OK) {
        print_response_line(&resp);
    } else {
        printf("ERR: home failed\n");
    }
    return 0;
}

static int cmd_can_status_like(const char *target)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            can_master_response_t resp = {0};
            esp_err_t err = can_master_request_status(CONFIGURED_NODE_IDS[i], &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err == ESP_OK) {
                can_master_work_offset_t work_offset = {0};
                err = can_master_wait_work_offset(CONFIGURED_NODE_IDS[i], &work_offset, CAN_MASTER_RESPONSE_TIMEOUT_MS);
                if (err == ESP_OK) {
                    printf("node=%u armed=%s wcofs=",
                           (unsigned)CONFIGURED_NODE_IDS[i],
                           ((resp.state_flags & CAN_STATE_FLAG_ARMED) != 0U) ? "yes" : "no");
                    print_work_offset_values(work_offset.work_offset_0p1mm);
                    printf("\n");
                } else {
                    printf("node=%u armed=%s wcofs=[n/a n/a n/a]\n",
                           (unsigned)CONFIGURED_NODE_IDS[i],
                           ((resp.state_flags & CAN_STATE_FLAG_ARMED) != 0U) ? "yes" : "no");
                }
            } else {
                printf("ERR: node=%u status failed\n", (unsigned)CONFIGURED_NODE_IDS[i]);
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = can_master_request_status(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err == ESP_OK) {
        can_master_work_offset_t work_offset = {0};
        err = can_master_wait_work_offset(node, &work_offset, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err == ESP_OK) {
            printf("node=%u armed=%s wcofs=",
                   (unsigned)node,
                   ((resp.state_flags & CAN_STATE_FLAG_ARMED) != 0U) ? "yes" : "no");
            print_work_offset_values(work_offset.work_offset_0p1mm);
            printf("\n");
        } else {
            printf("node=%u armed=%s wcofs=[n/a n/a n/a]\n",
                   (unsigned)node,
                   ((resp.state_flags & CAN_STATE_FLAG_ARMED) != 0U) ? "yes" : "no");
        }
    } else {
        printf("ERR: status failed\n");
    }
    return 0;
}

static int cmd_can_slot_like(const char *op, const char *target, const char *slot_arg)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    uint8_t slot = 0;
    if (!parse_slot_arg(slot_arg, &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }
    const bool do_prepare = (strcmp(op, "prepare") == 0);
    const bool do_run = (strcmp(op, "run") == 0);
    const bool do_upload = (strcmp(op, "upload") == 0);
    const bool do_delete = (strcmp(op, "delete") == 0);

    if (do_run && all) {
        printf("WARN: run all starts nodes sequentially, not synchronously. Use prepare all <slot> + sync for synchronized start.\n");
    }

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            const uint8_t current_node = CONFIGURED_NODE_IDS[i];
            can_master_response_t resp = {0};
            esp_err_t err = ESP_FAIL;

            if (do_prepare) {
                err = can_master_prepare_slot(current_node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            } else if (do_run) {
                err = can_master_program_run(current_node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            } else if (do_upload) {
                err = can_master_upload_gcode_program(current_node,
                                                      slot,
                                                      (const uint8_t *)DEFAULT_GCODE,
                                                      strlen(DEFAULT_GCODE),
                                                      CAN_MASTER_RESPONSE_TIMEOUT_MS);
            } else if (do_delete) {
                err = can_master_program_delete(current_node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            }

            if (err == ESP_OK) {
                if (do_upload) {
                    printf("OK: uploaded default G-code to node=%u slot=%u\n",
                           (unsigned)current_node,
                           (unsigned)slot);
                } else {
                    print_response_line(&resp);
                }
            } else {
                printf("ERR: node=%u %s failed\n", (unsigned)current_node, op);
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = ESP_FAIL;
    if (do_prepare) {
        err = can_master_prepare_slot(node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    } else if (do_run) {
        err = can_master_program_run(node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    } else if (do_upload) {
        err = can_master_upload_gcode_program(node,
                                              slot,
                                              (const uint8_t *)DEFAULT_GCODE,
                                              strlen(DEFAULT_GCODE),
                                              CAN_MASTER_RESPONSE_TIMEOUT_MS);
    } else if (do_delete) {
        err = can_master_program_delete(node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    }

    if (err == ESP_OK) {
        if (do_upload) {
            printf("OK: uploaded default G-code to node=%u slot=%u\n", (unsigned)node, (unsigned)slot);
        } else {
            print_response_line(&resp);
        }
    } else {
        printf("ERR: %s failed\n", op);
    }
    return 0;
}

static int cmd_can_upload_file(const char *target, const char *slot_arg, const char *file_arg)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(target, &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    uint8_t slot = 0;
    if (!parse_slot_arg(slot_arg, &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }

    char path[160] = {0};
    if (!normalize_spiffs_path(file_arg, path, sizeof(path))) {
        printf("ERR: invalid path\n");
        return 0;
    }

    uint8_t *file_data = NULL;
    size_t file_size = 0;
    esp_err_t load_err = load_file_from_spiffs(path, &file_data, &file_size);
    if (load_err != ESP_OK) {
        printf("ERR: read %s failed (%s)\n", path, esp_err_to_name(load_err));
        printf("Hint: put your file into SPIFFS image under spiffs/data.\n");
        return 0;
    }

    printf("Loaded %s (%u bytes)\n", path, (unsigned)file_size);

    if (all) {
        for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
            const uint8_t current_node = CONFIGURED_NODE_IDS[i];
            esp_err_t err = can_master_upload_gcode_program(current_node,
                                                            slot,
                                                            file_data,
                                                            file_size,
                                                            CAN_MASTER_RESPONSE_TIMEOUT_MS);
            if (err == ESP_OK) {
                printf("OK: uploaded %s to node=%u slot=%u\n", path, (unsigned)current_node, (unsigned)slot);
            } else {
                printf("ERR: node=%u upload_file failed\n", (unsigned)current_node);
            }
        }
    } else {
        esp_err_t err = can_master_upload_gcode_program(node,
                                                        slot,
                                                        file_data,
                                                        file_size,
                                                        CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err == ESP_OK) {
            printf("OK: uploaded %s to node=%u slot=%u\n", path, (unsigned)node, (unsigned)slot);
        } else {
            printf("ERR: upload_file failed\n");
        }
    }

    free(file_data);
    return 0;
}

static int cmd_can_run_sync(int argc, char **argv)
{
    uint8_t node = 0;
    bool all = false;
    if (!parse_node_or_all(argv[2], &node, &all)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    if (!all) {
        printf("ERR: run_sync supports only `all`; use `can run <node> <slot>` for one node\n");
        return 0;
    }

    uint8_t slot = RUN_SYNC_DEFAULT_SLOT;
    if (argc == 5 && !parse_slot_arg(argv[4], &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }

    char path[160] = {0};
    if (!normalize_spiffs_path(argv[3], path, sizeof(path))) {
        printf("ERR: invalid path\n");
        return 0;
    }

    uint8_t *file_data = NULL;
    size_t file_size = 0;
    esp_err_t err = load_file_from_spiffs(path, &file_data, &file_size);
    if (err != ESP_OK) {
        printf("ERR: read %s failed (%s)\n", path, esp_err_to_name(err));
        printf("Hint: put your file into SPIFFS image under spiffs/data.\n");
        return 0;
    }

    printf("RUN_SYNC: loaded %s (%u bytes), slot=%u\n",
           path,
           (unsigned)file_size,
           (unsigned)slot);

    can_master_clear_pending_responses();
    (void)can_master_stop_all();
    vTaskDelay(pdMS_TO_TICKS(100));
    can_master_clear_pending_responses();

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t current_node = CONFIGURED_NODE_IDS[i];
        can_master_response_t resp = {0};

        printf("RUN_SYNC: arm node=%u\n", (unsigned)current_node);
        err = can_master_arm(current_node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (resp.node_id != 0U) {
            print_response_line(&resp);
        }
        if (err != ESP_OK) {
            printf("ERR: node=%u arm failed (%s)\n", (unsigned)current_node, esp_err_to_name(err));
            free(file_data);
            return 0;
        }
    }

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t current_node = CONFIGURED_NODE_IDS[i];

        can_master_clear_pending_responses();
        printf("RUN_SYNC: upload node=%u slot=%u\n", (unsigned)current_node, (unsigned)slot);
        err = can_master_upload_gcode_program(current_node,
                                              slot,
                                              file_data,
                                              file_size,
                                              CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            printf("ERR: node=%u upload failed (%s)\n", (unsigned)current_node, esp_err_to_name(err));
            free(file_data);
            return 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
    }

    free(file_data);

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t current_node = CONFIGURED_NODE_IDS[i];
        can_master_response_t resp = {0};

        can_master_clear_pending_responses();
        printf("RUN_SYNC: prepare node=%u slot=%u\n", (unsigned)current_node, (unsigned)slot);
        err = can_master_prepare_slot(current_node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (resp.node_id != 0U) {
            print_response_line(&resp);
        }
        if (err != ESP_OK) {
            printf("ERR: node=%u prepare failed (%s)\n", (unsigned)current_node, esp_err_to_name(err));
            return 0;
        }
    }

    can_master_clear_pending_responses();
    printf("RUN_SYNC: sync start\n");
    err = can_master_sync_start_all();
    printf(err == ESP_OK ? "OK: run_sync started\n" : "ERR: sync start failed\n");
    return 0;
}

static int cmd_can_seq(int argc, char **argv)
{
    uint8_t slot = 0;
    char slot_text[8] = "0";
    if (argc == 3) {
        char *end = NULL;
        unsigned long parsed_slot = strtoul(argv[2], &end, 0);
        if (end == argv[2] || *end != '\0' || parsed_slot >= CAN_PROGRAM_SLOT_COUNT) {
            printf("ERR: invalid slot\n");
            return 0;
        }
        slot = (uint8_t)parsed_slot;
    }
    snprintf(slot_text, sizeof(slot_text), "%u", (unsigned)slot);

    printf("SEQ: arm all\n");
    (void)cmd_can_arm_like(true, "all");
    printf("SEQ: upload all slot=%u\n", (unsigned)slot);
    (void)cmd_can_slot_like("upload", "all", slot_text);
    printf("SEQ: prepare all slot=%u\n", (unsigned)slot);
    (void)cmd_can_slot_like("prepare", "all", slot_text);
    printf("SEQ: sync\n");
    esp_err_t err = can_master_sync_start_all();
    printf(err == ESP_OK ? "OK: sync start sent\n" : "ERR: sync start failed\n");
    return 0;
}

static bool parse_sync_measure_duration_arg(const char *arg, uint32_t *out_duration_ms)
{
    if (arg == NULL || out_duration_ms == NULL) {
        return false;
    }

    char *end = NULL;
    unsigned long parsed_ms = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' ||
        parsed_ms < SYNC_MEASURE_MIN_DURATION_MS ||
        parsed_ms > SYNC_MEASURE_MAX_DURATION_MS) {
        return false;
    }

    *out_duration_ms = (uint32_t)parsed_ms;
    return true;
}

static esp_err_t sync_measure_upload_program_all(uint8_t slot, const char *label, const char *program)
{
    if (label == NULL || program == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t node_id = CONFIGURED_NODE_IDS[i];
        printf("MEASURE_SYNC: upload %s node=%u slot=%u\n",
               label,
               (unsigned)node_id,
               (unsigned)slot);

        can_master_clear_pending_responses();
        esp_err_t err = can_master_upload_gcode_program(node_id,
                                                        slot,
                                                        (const uint8_t *)program,
                                                        strlen(program),
                                                        CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            printf("ERR: node=%u upload %s failed (%s)\n",
                   (unsigned)node_id,
                   label,
                   esp_err_to_name(err));
            return err;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
    }

    return ESP_OK;
}

static esp_err_t sync_measure_prepare_all(uint8_t slot)
{
    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        const uint8_t node_id = CONFIGURED_NODE_IDS[i];
        can_master_response_t resp = {0};

        can_master_clear_pending_responses();
        printf("MEASURE_SYNC: prepare node=%u slot=%u\n", (unsigned)node_id, (unsigned)slot);
        esp_err_t err = can_master_prepare_slot(node_id, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (resp.node_id != 0U) {
            print_response_line(&resp);
        }
        if (err != ESP_OK) {
            printf("ERR: node=%u prepare failed (%s)\n", (unsigned)node_id, esp_err_to_name(err));
            return err;
        }
    }

    return ESP_OK;
}

static int cmd_can_measure_sync(int argc, char **argv)
{
    uint32_t duration_ms = SYNC_MEASURE_DEFAULT_DURATION_MS;
    if (argc == 3 && !parse_sync_measure_duration_arg(argv[2], &duration_ms)) {
        printf("ERR: invalid duration_ms, use %u..%u\n",
               (unsigned)SYNC_MEASURE_MIN_DURATION_MS,
               (unsigned)SYNC_MEASURE_MAX_DURATION_MS);
        return 0;
    }

    const unsigned gripper_angle_deg = s_sync_measure_close_next ? 0U : 90U;
    char measure_gcode[SYNC_MEASURE_GCODE_BUFFER_SIZE] = {0};
    const uint32_t dwell_ms = duration_ms + SYNC_MEASURE_STOP_MARGIN_MS;
    int written = snprintf(measure_gcode,
                           sizeof(measure_gcode),
                           "G21\n"
                           "G90\n"
                           "M280 S%u\n"
                           "G4 P%u\n"
                           "M30\n",
                           gripper_angle_deg,
                           (unsigned)dwell_ms);
    if (written <= 0 || (size_t)written >= sizeof(measure_gcode)) {
        printf("ERR: measure G-code build failed\n");
        return 0;
    }

    printf("MEASURE_SYNC: one-shot setup uses slot=%u and overwrites it\n", (unsigned)SYNC_MEASURE_SLOT);
    printf("MEASURE_SYNC: target gripper angle S%u (next run toggles)\n", gripper_angle_deg);

    esp_err_t err = ESP_OK;

    can_master_clear_pending_responses();
    (void)can_master_stop_all();
    vTaskDelay(pdMS_TO_TICKS(100));
    can_master_clear_pending_responses();

    printf("MEASURE_SYNC: broadcast ARM\n");
    err = can_master_arm_all();
    if (err != ESP_OK) {
        printf("ERR: arm all failed (%s)\n", esp_err_to_name(err));
        return 0;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    can_master_clear_pending_responses();

    err = sync_measure_upload_program_all(SYNC_MEASURE_SLOT, "measure", measure_gcode);
    if (err != ESP_OK) {
        return 0;
    }

    err = sync_measure_prepare_all(SYNC_MEASURE_SLOT);
    if (err != ESP_OK) {
        return 0;
    }

    can_master_clear_pending_responses();
    printf("MEASURE_SYNC: t=0 ms send SYNC_START\n");
    err = can_master_sync_start_all();
    if (err != ESP_OK) {
        printf("ERR: sync start failed (%s)\n", esp_err_to_name(err));
        return 0;
    }

    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    printf("MEASURE_SYNC: t=%u ms send DISARM\n", (unsigned)duration_ms);
    err = can_master_disarm_all();
    vTaskDelay(pdMS_TO_TICKS(50));
    can_master_clear_pending_responses();
    if (err == ESP_OK) {
        s_sync_measure_close_next = !s_sync_measure_close_next;
    }
    printf(err == ESP_OK ? "OK: measure_sync finished\n" : "ERR: disarm failed\n");
    return 0;
}

static esp_err_t relay_execute_cycles(uint32_t cycles)
{
    static const relay_step_t relay_steps[] = {
        { CAN_MASTER_NODE2_ID, RELAY_NODE2_FORWARD_SLOT, "node2 slot0: HOME -> pick A -> place B -> HOME" },
        { CAN_MASTER_NODE1_ID, RELAY_NODE1_FORWARD_SLOT, "node1 slot0: HOME -> pick B -> place C -> HOME" },
        { CAN_MASTER_NODE1_ID, RELAY_NODE1_RETURN_SLOT, "node1 slot1: pick C -> place B -> HOME" },
        { CAN_MASTER_NODE2_ID, RELAY_NODE2_RETURN_SLOT, "node2 slot1: pick B -> place A -> HOME" },
    };

    printf("RELAY: arm + home node=%u\n", (unsigned)CAN_MASTER_NODE1_ID);
    esp_err_t err = relay_arm_and_home_node(CAN_MASTER_NODE1_ID);
    if (err != ESP_OK) {
        return err;
    }

    printf("RELAY: arm + home node=%u\n", (unsigned)CAN_MASTER_NODE2_ID);
    err = relay_arm_and_home_node(CAN_MASTER_NODE2_ID);
    if (err != ESP_OK) {
        return err;
    }

    err = relay_broadcast_home_all();
    if (err != ESP_OK) {
        return err;
    }

    for (uint32_t cycle = 0; cycle < cycles; cycle++) {
        if (relay_stop_requested()) {
            return ESP_ERR_INVALID_STATE;
        }

        relay_set_cycle_progress(cycle + 1U);
        printf("RELAY: cycle %" PRIu32 "/%" PRIu32 "\n", cycle + 1U, cycles);
        for (size_t i = 0; i < sizeof(relay_steps) / sizeof(relay_steps[0]); i++) {
            err = relay_run_step(&relay_steps[i]);
            if (err != ESP_OK) {
                return relay_stop_requested() ? ESP_ERR_INVALID_STATE : err;
            }
        }
    }

    printf("OK: relay orchestrator finished %" PRIu32 " cycle(s)\n", cycles);
    return ESP_OK;
}

static void relay_task(void *arg)
{
    const uint32_t cycles = (uint32_t)(uintptr_t)arg;
    const esp_err_t err = relay_execute_cycles(cycles);

    if (err == ESP_ERR_INVALID_STATE && relay_stop_requested()) {
        printf("RELAY: stopped\n");
    } else if (err != ESP_OK) {
        printf("ERR: relay aborted (%s)\n", esp_err_to_name(err));
    }

    taskENTER_CRITICAL(&s_relay_mux);
    s_relay_running = false;
    s_relay_stop_requested = false;
    s_relay_target_cycles = 0;
    s_relay_active_cycle = 0;
    taskEXIT_CRITICAL(&s_relay_mux);

    vTaskDelete(NULL);
}

static int cmd_can_relay_status(void)
{
    if (!relay_is_running()) {
        printf("RELAY: idle\n");
        return 0;
    }

    uint32_t active_cycle = 0;
    uint32_t target_cycles = 0;
    relay_get_progress(&active_cycle, &target_cycles);
    printf("RELAY: running cycle=%" PRIu32 "/%" PRIu32 "%s\n",
           active_cycle,
           target_cycles,
           relay_stop_requested() ? " stop_requested=yes" : "");
    return 0;
}

static int cmd_can_relay(int argc, char **argv)
{
    if (argc == 3 && strcmp(argv[2], "status") == 0) {
        return cmd_can_relay_status();
    }

    if (argc == 3 && strcmp(argv[2], "stop") == 0) {
        if (!relay_is_running()) {
            printf("RELAY: idle\n");
            return 0;
        }

        relay_request_stop();
        can_master_clear_pending_responses();
        esp_err_t err = can_master_stop_all();
        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
        printf(err == ESP_OK ? "OK: relay stop requested\n" : "ERR: relay stop failed\n");
        return 0;
    }

    uint32_t cycles = 1;
    if (argc == 3 && !parse_cycle_count_arg(argv[2], &cycles)) {
        printf("ERR: invalid cycle count\n");
        return 0;
    }

    if (relay_is_running()) {
        printf("ERR: relay is already running\n");
        return 0;
    }

    taskENTER_CRITICAL(&s_relay_mux);
    s_relay_running = true;
    s_relay_stop_requested = false;
    s_relay_target_cycles = cycles;
    s_relay_active_cycle = 0;
    taskEXIT_CRITICAL(&s_relay_mux);

    BaseType_t created = xTaskCreate(relay_task,
                                     "relay_task",
                                     RELAY_TASK_STACK_SIZE,
                                     (void *)(uintptr_t)cycles,
                                     RELAY_TASK_PRIORITY,
                                     NULL);
    if (created != pdPASS) {
        taskENTER_CRITICAL(&s_relay_mux);
        s_relay_running = false;
        s_relay_stop_requested = false;
        s_relay_target_cycles = 0;
        s_relay_active_cycle = 0;
        taskEXIT_CRITICAL(&s_relay_mux);
        printf("ERR: relay task start failed\n");
        return 0;
    }

    printf("OK: relay started (%" PRIu32 " cycle(s)); use `can relay status` or `can stop`\n", cycles);
    return 0;
}

static int cmd_can(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage:\n");
        printf("  can init\n");
        printf("  can nodes\n");
        printf("  can arm <node|all>\n");
        printf("  can disarm <node|all>\n");
        printf("  can home <node|all>\n");
        printf("  can status <node|all>\n");
        printf("  can sensors <node|all>\n");
        printf("  can prepare <node|all> <slot>\n");
        printf("  can run <node|all> <slot>\n");
        printf("  can run_sync all <path> [slot]   (loads /spiffs/<path>, default slot 0)\n");
        printf("  can delete <node|all> <slot>\n");
        printf("  can stop\n");
        printf("  can sync\n");
        printf("  can measure_sync [duration_ms]\n");
        printf("  can upload <node|all> <slot>   (uploads DEFAULT_GCODE)\n");
        printf("  can upload_file <node|all> <slot> <path>   (loads /spiffs/<path>)\n");
        printf("  can seq [slot]\n");
        printf("  can relay [cycles]\n");
        printf("  can relay status\n");
        printf("  can relay stop\n");
        return 0;
    }

    if (strcmp(argv[1], "nodes") == 0) {
        return cmd_can_nodes();
    }

    if (strcmp(argv[1], "stop") == 0) {
        if (relay_is_running()) {
            relay_request_stop();
        }

        can_master_clear_pending_responses();
        esp_err_t err = can_master_stop_all();
        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
        printf(err == ESP_OK
                   ? (relay_is_running() ? "OK: stop sent, relay stop requested\n" : "OK: stop sent\n")
                   : "ERR: stop failed\n");
        return 0;
    }

    if (strcmp(argv[1], "relay") == 0) {
        if (argc != 2 && argc != 3) {
            printf("Usage: can relay [cycles]\n");
            printf("       can relay status\n");
            printf("       can relay stop\n");
            return 0;
        }
        return cmd_can_relay(argc, argv);
    }

    if (relay_is_running()) {
        printf("ERR: relay is running; use `can relay status` or `can stop`\n");
        return 0;
    }

    if (strcmp(argv[1], "init") == 0) {
        esp_err_t err = can_master_init();
        if (err == ESP_OK) {
            printf("OK: can init\n");
        } else {
            printf("ERR: can init failed (%s)\n", esp_err_to_name(err));
        }
        return 0;
    }

    if (strcmp(argv[1], "sync") == 0) {
        esp_err_t err = can_master_sync_start_all();
        printf(err == ESP_OK ? "OK: sync start sent\n" : "ERR: sync start failed\n");
        return 0;
    }

    if (strcmp(argv[1], "measure_sync") == 0) {
        if (argc != 2 && argc != 3) {
            printf("Usage: can measure_sync [duration_ms]\n");
            return 0;
        }
        return cmd_can_measure_sync(argc, argv);
    }

    if (strcmp(argv[1], "run_sync") == 0) {
        if (argc != 4 && argc != 5) {
            printf("Usage: can run_sync all <path> [slot]\n");
            return 0;
        }
        return cmd_can_run_sync(argc, argv);
    }

    if (strcmp(argv[1], "seq") == 0) {
        if (argc != 2 && argc != 3) {
            printf("Usage: can seq [slot]\n");
            return 0;
        }
        return cmd_can_seq(argc, argv);
    }

    if (strcmp(argv[1], "arm") == 0 || strcmp(argv[1], "disarm") == 0) {
        if (argc != 3) {
            printf("Usage: can %s <node|all>\n", argv[1]);
            return 0;
        }
        return cmd_can_arm_like(strcmp(argv[1], "arm") == 0, argv[2]);
    }

    if (strcmp(argv[1], "home") == 0) {
        if (argc != 3) {
            printf("Usage: can home <node|all>\n");
            return 0;
        }
        return cmd_can_home_like(argv[2]);
    }

    if (strcmp(argv[1], "status") == 0) {
        if (argc != 3) {
            printf("Usage: can status <node|all>\n");
            return 0;
        }
        return cmd_can_status_like(argv[2]);
    }

    if (strcmp(argv[1], "sensors") == 0) {
        if (argc != 3) {
            printf("Usage: can sensors <node|all>\n");
            return 0;
        }
        return cmd_can_sensors_like(argv[2]);
    }

    if (strcmp(argv[1], "prepare") == 0 || strcmp(argv[1], "run") == 0 || strcmp(argv[1], "delete") == 0) {
        if (argc != 4) {
            printf("Usage: can %s <node|all> <slot>\n", argv[1]);
            return 0;
        }
        return cmd_can_slot_like(argv[1], argv[2], argv[3]);
    }

    if (strcmp(argv[1], "upload") == 0) {
        if (argc != 4) {
            printf("Usage: can upload <node|all> <slot>\n");
            return 0;
        }
        return cmd_can_slot_like("upload", argv[2], argv[3]);
    }

    if (strcmp(argv[1], "upload_file") == 0) {
        if (argc != 5) {
            printf("Usage: can upload_file <node|all> <slot> <path>\n");
            return 0;
        }
        return cmd_can_upload_file(argv[2], argv[3], argv[4]);
    }

    printf("Unknown can subcommand\n");
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        { .command = "can", .help = "CAN master control", .func = &cmd_can },
    };

    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        esp_console_cmd_t c = cmds[i];
        c.hint = NULL;
        c.argtable = NULL;
        ESP_ERROR_CHECK(esp_console_cmd_register(&c));
    }
}

void cmd_control_start(void)
{
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = ">> ";
    repl_cfg.max_history_len = 10;
    repl_cfg.max_cmdline_length = 128;
    repl_cfg.task_stack_size = 4096;

    esp_err_t err = ESP_OK;

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t usb_cfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    err = esp_console_new_repl_usb_serial_jtag(&usb_cfg, &repl_cfg, &s_repl);
#else
    esp_console_dev_uart_config_t uart_cfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    err = esp_console_new_repl_uart(&uart_cfg, &repl_cfg, &s_repl);
#endif
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "console REPL create failed: %s", esp_err_to_name(err));
        return;
    }

    register_commands();

    linenoiseSetDumbMode(1);
    linenoiseSetMultiLine(0);

    err = esp_console_start_repl(s_repl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "console REPL start failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "console REPL started");
}
