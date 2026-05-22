#include "cmd_control.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
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

static const uint8_t CONFIGURED_NODE_IDS[] = {CAN_MASTER_NODE1_ID, CAN_MASTER_NODE2_ID};
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define CONFIGURED_NODE_COUNT ARRAY_SIZE(CONFIGURED_NODE_IDS)

typedef esp_err_t (*node_response_fn_t)(uint8_t node_id, can_master_response_t *out, uint32_t timeout_ms);

typedef enum {
    SLOT_OP_PREPARE,
    SLOT_OP_RUN,
    SLOT_OP_UPLOAD,
    SLOT_OP_DELETE,
} slot_op_t;

typedef struct {
    uint8_t ids[CONFIGURED_NODE_COUNT];
    size_t count;
    bool all;
} node_target_t;

typedef struct {
    uint8_t node_id;
    uint8_t slot;
    const char *label;
} relay_step_t;

typedef struct {
    uint8_t node_id;
    uint8_t slot;
    const char *path;
} relay_upload_entry_t;

static const char *TAG = "cmd";
static volatile bool s_relay_running;
static volatile bool s_relay_stop_requested;
static volatile uint32_t s_relay_target_cycles;
static volatile uint32_t s_relay_active_cycle;

static portMUX_TYPE s_relay_mux = portMUX_INITIALIZER_UNLOCKED;

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

static const char CAN_USAGE[] =
    "Usage:\n"
    "  can init\n"
    "  can nodes\n"
    "  can arm <node|all>\n"
    "  can disarm <node|all>\n"
    "  can home <node|all>\n"
    "  can status <node|all>\n"
    "  can sensors <node|all>\n"
    "  can prepare <node|all> <slot>\n"
    "  can run <node|all> <slot>\n"
    "  can run_sync all <path> [slot]   (loads /spiffs/<path>, default slot 0)\n"
    "  can delete <node|all> <slot>\n"
    "  can stop\n"
    "  can sync\n"
    "  can upload <node|all> <slot>   (uploads DEFAULT_GCODE)\n"
    "  can upload_file <node|all> <slot> <path>   (loads /spiffs/<path>)\n"
    "  can seq [slot]\n"
    "  can relay [cycles]\n"
    "  can relay status\n"
    "  can relay stop\n";

static bool parse_u32_arg(const char *arg, uint32_t min, uint32_t max, uint32_t *out)
{
    if (arg == NULL || out == NULL) return false;
    char *end = NULL;
    unsigned long value = strtoul(arg, &end, 0);
    if (end == arg || *end != '\0' || value < min || value > max) return false;
    *out = (uint32_t)value;
    return true;
}

static bool parse_slot_arg(const char *arg, uint8_t *out_slot)
{
    uint32_t slot = 0;
    if (!parse_u32_arg(arg, 0, CAN_PROGRAM_SLOT_COUNT - 1U, &slot)) return false;
    *out_slot = (uint8_t)slot;
    return true;
}

static bool parse_node_target(const char *arg, node_target_t *target)
{
    if (arg == NULL || target == NULL) return false;
    memset(target, 0, sizeof(*target));

    if (strcmp(arg, "all") == 0) {
        memcpy(target->ids, CONFIGURED_NODE_IDS, sizeof(CONFIGURED_NODE_IDS));
        target->count = CONFIGURED_NODE_COUNT;
        target->all = true;
        return true;
    }

    uint32_t node_id = 0;
    if (!parse_u32_arg(arg, 1U, CAN_NODE_ID_MAX, &node_id)) return false;
    target->ids[0] = (uint8_t)node_id;
    target->count = 1U;
    return true;
}

static bool normalize_spiffs_path(const char *input, char *output, size_t output_size)
{
    if (input == NULL || output == NULL || output_size == 0U) return false;
    const char *fmt = (strncmp(input, "/spiffs/", 8) == 0) ? "%s" : (input[0] == '/') ? "/spiffs%s" : "/spiffs/%s";
    int written = snprintf(output, output_size, fmt, input);
    return written > 0 && (size_t)written < output_size;
}

static esp_err_t load_file_from_spiffs(const char *path, uint8_t **out_data, size_t *out_size)
{
    if (path == NULL || out_data == NULL || out_size == NULL) return ESP_ERR_INVALID_ARG;

    FILE *f = fopen(path, "rb");
    if (f == NULL) return ESP_ERR_NOT_FOUND;

    esp_err_t err = ESP_FAIL;
    uint8_t *data = NULL;
    long file_size = 0;

    if (fseek(f, 0, SEEK_END) != 0) goto done;
    file_size = ftell(f);
    if (file_size <= 0 || file_size > 0xFFFFL) {
        err = (file_size <= 0) ? ESP_ERR_INVALID_SIZE : ESP_ERR_INVALID_ARG;
        goto done;
    }
    rewind(f);

    data = (uint8_t *)malloc((size_t)file_size);
    if (data == NULL) {
        err = ESP_ERR_NO_MEM;
        goto done;
    }
    if (fread(data, 1, (size_t)file_size, f) != (size_t)file_size) goto done;

    *out_data = data;
    *out_size = (size_t)file_size;
    data = NULL;
    err = ESP_OK;

done:
    free(data);
    fclose(f);
    return err;
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

static void print_response_line(const can_master_response_t *resp)
{
    if (resp == NULL) return;
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

static void print_status_snapshot(const can_master_status_t *status)
{
    if (status == NULL) return;
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

static void print_fixed_0p1_i16(int16_t value, int16_t invalid_value)
{
    if (value == invalid_value) {
        printf("n/a");
        return;
    }

    int32_t signed_value = value;
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

static void relay_set_state(bool running, bool stop_requested, uint32_t target_cycles, uint32_t active_cycle)
{
    taskENTER_CRITICAL(&s_relay_mux);
    s_relay_running = running;
    s_relay_stop_requested = stop_requested;
    s_relay_target_cycles = target_cycles;
    s_relay_active_cycle = active_cycle;
    taskEXIT_CRITICAL(&s_relay_mux);
}

static bool relay_is_running(void)
{
    taskENTER_CRITICAL(&s_relay_mux);
    bool running = s_relay_running;
    taskEXIT_CRITICAL(&s_relay_mux);
    return running;
}

static bool relay_stop_requested(void)
{
    taskENTER_CRITICAL(&s_relay_mux);
    bool stop_requested = s_relay_stop_requested;
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
    if (out_active_cycle != NULL) *out_active_cycle = s_relay_active_cycle;
    if (out_target_cycles != NULL) *out_target_cycles = s_relay_target_cycles;
    taskEXIT_CRITICAL(&s_relay_mux);
}

static esp_err_t stop_all_flushed(uint32_t settle_ms)
{
    can_master_clear_pending_responses();
    esp_err_t err = can_master_stop_all();
    vTaskDelay(pdMS_TO_TICKS(settle_ms));
    can_master_clear_pending_responses();
    return err;
}

static esp_err_t read_cached_status(uint8_t node_id, can_master_status_t *out_status)
{
    if (out_status == NULL) return ESP_ERR_INVALID_ARG;
    return can_master_node_is_online(node_id, CAN_MASTER_NODE_ONLINE_TIMEOUT_MS)
               ? can_master_get_last_status(node_id, out_status)
               : ESP_ERR_NOT_FOUND;
}

static bool status_is_ready_for_program(const can_master_status_t *status)
{
    return status != NULL &&
           status->robot_state == ROBOT_STATE_READY &&
           status->active_slot == CAN_SLOT_NONE &&
           (status->state_flags & CAN_STATE_FLAG_ARMED) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_REFERENCED) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_TCP_EST) != 0U &&
           (status->state_flags & CAN_STATE_FLAG_PROGRAM_RUNNING) == 0U &&
           (status->state_flags & CAN_STATE_FLAG_ERROR) == 0U;
}

static bool status_is_running_expected_slot(const can_master_status_t *status, uint8_t slot)
{
    return status != NULL &&
           (status->robot_state == ROBOT_STATE_RUNNING ||
            (status->state_flags & CAN_STATE_FLAG_PROGRAM_RUNNING) != 0U ||
            status->active_slot == slot);
}

static esp_err_t wait_for_node_ready(uint8_t node_id, uint8_t expected_slot, uint32_t timeout_ms)
{
    const TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool saw_running = false;
    esp_err_t last_err = ESP_OK;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - start_tick) >= timeout_ticks) return (last_err != ESP_OK) ? last_err : ESP_ERR_TIMEOUT;
        if (relay_stop_requested()) return ESP_ERR_INVALID_STATE;

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
        if (expected_slot != CAN_SLOT_NONE && status_is_running_expected_slot(&status, expected_slot)) saw_running = true;
        if (status_is_ready_for_program(&status) &&
            (saw_running || (uint32_t)pdTICKS_TO_MS(now - start_tick) >= RELAY_READY_GRACE_MS)) {
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(RELAY_STATUS_POLL_MS));
    }
}

static esp_err_t relay_send_response_cmd(uint8_t node_id, const char *name, node_response_fn_t fn)
{
    if (relay_stop_requested()) return ESP_ERR_INVALID_STATE;

    can_master_response_t resp = {0};
    can_master_clear_pending_responses();
    esp_err_t err = fn(node_id, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (resp.node_id != 0U) print_response_line(&resp);
    if (err != ESP_OK) printf("ERR: node=%u %s failed (%s)\n", (unsigned)node_id, name, esp_err_to_name(err));
    return err;
}

static esp_err_t relay_arm_and_home_node(uint8_t node_id)
{
    esp_err_t err = relay_send_response_cmd(node_id, "arm", can_master_arm);
    if (err != ESP_OK) return err;

    err = relay_send_response_cmd(node_id, "home", can_master_home);
    if (err != ESP_OK) return err;

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
    if (relay_stop_requested()) return ESP_ERR_INVALID_STATE;

    can_master_clear_pending_responses();
    printf("RELAY: broadcast HOME to both nodes\n");
    esp_err_t err = can_master_home_broadcast_all();
    if (err != ESP_OK) {
        printf("ERR: broadcast HOME failed (%s)\n", esp_err_to_name(err));
        return err;
    }

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        err = wait_for_node_ready(node, CAN_SLOT_NONE, RELAY_HOME_SETTLE_TIMEOUT_MS);
        if (err != ESP_OK) {
            printf("ERR: node=%u broadcast HOME did not finish cleanly (%s)\n", (unsigned)node, esp_err_to_name(err));
            return err;
        }
    }

    printf("OK: synchronized HOME reference finished on both nodes\n");
    return ESP_OK;
}

static esp_err_t relay_run_step(const relay_step_t *step)
{
    if (step == NULL) return ESP_ERR_INVALID_ARG;
    if (relay_stop_requested()) return ESP_ERR_INVALID_STATE;

    can_master_response_t resp = {0};
    printf("RELAY: run node=%u slot=%u (%s)\n", (unsigned)step->node_id, (unsigned)step->slot, step->label);
    can_master_clear_pending_responses();

    esp_err_t err = can_master_program_run(step->node_id, step->slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (resp.node_id != 0U) print_response_line(&resp);
    if (err != ESP_OK) {
        printf("ERR: node=%u slot=%u run failed (%s)\n",
               (unsigned)step->node_id,
               (unsigned)step->slot,
               esp_err_to_name(err));
        return err;
    }

    err = wait_for_node_ready(step->node_id, step->slot, RELAY_READY_TIMEOUT_MS);
    if (err != ESP_OK) {
        printf("ERR: node=%u slot=%u did not finish cleanly (%s)\n",
               (unsigned)step->node_id,
               (unsigned)step->slot,
               esp_err_to_name(err));
        return err;
    }

    printf("OK: node=%u slot=%u finished (%s)\n", (unsigned)step->node_id, (unsigned)step->slot, step->label);
    return ESP_OK;
}

static int cmd_can_nodes(void)
{
    printf("Configured nodes (online timeout %u ms):\n", (unsigned)CAN_MASTER_NODE_ONLINE_TIMEOUT_MS);
    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        printf("  node=%u %s\n",
               (unsigned)node,
               can_master_node_is_online(node, CAN_MASTER_NODE_ONLINE_TIMEOUT_MS) ? "online" : "offline");
    }
    return 0;
}

static int cmd_can_response_like(const char *name, const char *target_arg, node_response_fn_t fn)
{
    node_target_t target;
    if (!parse_node_target(target_arg, &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    for (size_t i = 0; i < target.count; i++) {
        uint8_t node = target.ids[i];
        can_master_response_t resp = {0};
        esp_err_t err = fn(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err == ESP_OK) {
            print_response_line(&resp);
        } else if (target.all) {
            printf("ERR: node=%u %s failed\n", (unsigned)node, name);
        } else {
            printf("ERR: %s failed\n", name);
        }
    }
    return 0;
}

static int cmd_can_sensors_like(const char *target_arg)
{
    node_target_t target;
    if (!parse_node_target(target_arg, &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    for (size_t i = 0; i < target.count; i++) {
        uint8_t node = target.ids[i];
        can_master_response_t resp = {0};
        esp_err_t err = can_master_request_status(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            if (target.all) {
                printf("ERR: node=%u sensors failed (%s)\n", (unsigned)node, esp_err_to_name(err));
            } else {
                printf("ERR: sensors failed (%s)\n", esp_err_to_name(err));
            }
            continue;
        }

        can_master_sensor_values_t sensors = {0};
        err = can_master_wait_sensor_values(node, &sensors, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err == ESP_OK) {
            print_sensor_values_line(&sensors);
        } else {
            if (target.all) {
                printf("ERR: node=%u sensors failed (%s)\n", (unsigned)node, esp_err_to_name(err));
            } else {
                printf("ERR: sensors failed (%s)\n", esp_err_to_name(err));
            }
        }
    }
    return 0;
}

static int cmd_can_status_like(const char *target_arg)
{
    node_target_t target;
    if (!parse_node_target(target_arg, &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }

    for (size_t i = 0; i < target.count; i++) {
        uint8_t node = target.ids[i];
        can_master_response_t resp = {0};
        esp_err_t err = can_master_request_status(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            printf(target.all ? "ERR: node=%u status failed\n" : "ERR: status failed\n", (unsigned)node);
            continue;
        }

        can_master_work_offset_t work_offset = {0};
        err = can_master_wait_work_offset(node, &work_offset, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        printf("node=%u armed=%s wcofs=", (unsigned)node, (resp.state_flags & CAN_STATE_FLAG_ARMED) ? "yes" : "no");
        if (err == ESP_OK) {
            print_work_offset_values(work_offset.work_offset_0p1mm);
        } else {
            printf("[n/a n/a n/a]");
        }
        printf("\n");
    }
    return 0;
}

static esp_err_t run_slot_op(slot_op_t op, uint8_t node, uint8_t slot, can_master_response_t *resp)
{
    switch (op) {
        case SLOT_OP_PREPARE: return can_master_prepare_slot(node, slot, resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        case SLOT_OP_RUN: return can_master_program_run(node, slot, resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        case SLOT_OP_DELETE: return can_master_program_delete(node, slot, resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        case SLOT_OP_UPLOAD:
            return can_master_upload_gcode_program(node,
                                                   slot,
                                                   (const uint8_t *)DEFAULT_GCODE,
                                                   strlen(DEFAULT_GCODE),
                                                   CAN_MASTER_RESPONSE_TIMEOUT_MS);
        default: return ESP_ERR_INVALID_ARG;
    }
}

static int cmd_can_slot_like(slot_op_t op, const char *name, const char *target_arg, const char *slot_arg)
{
    node_target_t target;
    uint8_t slot = 0;
    if (!parse_node_target(target_arg, &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }
    if (!parse_slot_arg(slot_arg, &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }
    if (op == SLOT_OP_RUN && target.all) {
        printf("WARN: run all starts nodes sequentially, not synchronously. Use prepare all <slot> + sync for synchronized start.\n");
    }

    for (size_t i = 0; i < target.count; i++) {
        uint8_t node = target.ids[i];
        can_master_response_t resp = {0};
        esp_err_t err = run_slot_op(op, node, slot, &resp);
        if (err == ESP_OK && op == SLOT_OP_UPLOAD) {
            printf("OK: uploaded default G-code to node=%u slot=%u\n", (unsigned)node, (unsigned)slot);
        } else if (err == ESP_OK) {
            print_response_line(&resp);
        } else if (target.all) {
            printf("ERR: node=%u %s failed\n", (unsigned)node, name);
        } else {
            printf("ERR: %s failed\n", name);
        }
    }
    return 0;
}

static int cmd_can_upload_file(const char *target_arg, const char *slot_arg, const char *file_arg)
{
    node_target_t target;
    uint8_t slot = 0;
    char path[160] = {0};

    if (!parse_node_target(target_arg, &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }
    if (!parse_slot_arg(slot_arg, &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }
    if (!normalize_spiffs_path(file_arg, path, sizeof(path))) {
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

    printf("Loaded %s (%u bytes)\n", path, (unsigned)file_size);
    for (size_t i = 0; i < target.count; i++) {
        uint8_t node = target.ids[i];
        err = can_master_upload_gcode_program(node, slot, file_data, file_size, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err == ESP_OK) {
            printf("OK: uploaded %s to node=%u slot=%u\n", path, (unsigned)node, (unsigned)slot);
        } else {
            printf(target.all ? "ERR: node=%u upload_file failed\n" : "ERR: upload_file failed\n", (unsigned)node);
        }
    }

    free(file_data);
    return 0;
}

static esp_err_t upload_program_all(uint8_t slot, const char *label, const uint8_t *program, size_t program_size)
{
    if (label == NULL || program == NULL || program_size == 0U) return ESP_ERR_INVALID_ARG;

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        printf("%s: upload node=%u slot=%u\n", label, (unsigned)node, (unsigned)slot);
        can_master_clear_pending_responses();
        esp_err_t err = can_master_upload_gcode_program(node, slot, program, program_size, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (err != ESP_OK) {
            printf("ERR: node=%u upload failed (%s)\n", (unsigned)node, esp_err_to_name(err));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
    }
    return ESP_OK;
}

static esp_err_t prepare_all(uint8_t slot, const char *label)
{
    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        can_master_response_t resp = {0};
        can_master_clear_pending_responses();
        printf("%s: prepare node=%u slot=%u\n", label, (unsigned)node, (unsigned)slot);
        esp_err_t err = can_master_prepare_slot(node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (resp.node_id != 0U) print_response_line(&resp);
        if (err != ESP_OK) {
            printf("ERR: node=%u prepare failed (%s)\n", (unsigned)node, esp_err_to_name(err));
            return err;
        }
    }
    return ESP_OK;
}

static int cmd_can_run_sync(int argc, char **argv)
{
    node_target_t target;
    if (!parse_node_target(argv[2], &target)) {
        printf("ERR: invalid node target\n");
        return 0;
    }
    if (!target.all) {
        printf("ERR: run_sync supports only `all`; use `can run <node> <slot>` for one node\n");
        return 0;
    }

    uint8_t slot = RUN_SYNC_DEFAULT_SLOT;
    char path[160] = {0};
    if (argc == 5 && !parse_slot_arg(argv[4], &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }
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

    printf("RUN_SYNC: loaded %s (%u bytes), slot=%u\n", path, (unsigned)file_size, (unsigned)slot);
    (void)stop_all_flushed(100);

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        can_master_response_t resp = {0};
        printf("RUN_SYNC: arm node=%u\n", (unsigned)node);
        err = can_master_arm(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        if (resp.node_id != 0U) print_response_line(&resp);
        if (err != ESP_OK) {
            printf("ERR: node=%u arm failed (%s)\n", (unsigned)node, esp_err_to_name(err));
            free(file_data);
            return 0;
        }
    }

    err = upload_program_all(slot, "RUN_SYNC", file_data, file_size);
    free(file_data);
    if (err != ESP_OK) return 0;

    if (prepare_all(slot, "RUN_SYNC") != ESP_OK) return 0;

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
    if (argc == 3 && !parse_slot_arg(argv[2], &slot)) {
        printf("ERR: invalid slot\n");
        return 0;
    }
    snprintf(slot_text, sizeof(slot_text), "%u", (unsigned)slot);

    printf("SEQ: arm all\n");
    (void)cmd_can_response_like("arm", "all", can_master_arm);
    printf("SEQ: upload all slot=%u\n", (unsigned)slot);
    (void)cmd_can_slot_like(SLOT_OP_UPLOAD, "upload", "all", slot_text);
    printf("SEQ: prepare all slot=%u\n", (unsigned)slot);
    (void)cmd_can_slot_like(SLOT_OP_PREPARE, "prepare", "all", slot_text);
    printf("SEQ: sync\n");
    esp_err_t err = can_master_sync_start_all();
    printf(err == ESP_OK ? "OK: sync start sent\n" : "ERR: sync start failed\n");
    return 0;
}

static esp_err_t relay_upload_programs(void)
{
    static const relay_upload_entry_t uploads[] = {
        {CAN_MASTER_NODE1_ID, RELAY_FORWARD_SLOT, "/spiffs/node1_slot0.gcode"},
        {CAN_MASTER_NODE2_ID, RELAY_FORWARD_SLOT, "/spiffs/node2_slot0.gcode"},
        {CAN_MASTER_NODE1_ID, RELAY_RETURN_SLOT,  "/spiffs/node1_slot1.gcode"},
        {CAN_MASTER_NODE2_ID, RELAY_RETURN_SLOT,  "/spiffs/node2_slot1.gcode"},
    };

    for (size_t i = 0; i < ARRAY_SIZE(uploads); i++) {
        const relay_upload_entry_t *u = &uploads[i];
        uint8_t *data = NULL;
        size_t size = 0;
        esp_err_t err = load_file_from_spiffs(u->path, &data, &size);
        if (err != ESP_OK) {
            printf("ERR: relay upload: read %s failed (%s)\n", u->path, esp_err_to_name(err));
            return err;
        }
        printf("RELAY: upload node=%u slot=%u from %s (%u B)\n",
               (unsigned)u->node_id, (unsigned)u->slot, u->path, (unsigned)size);
        can_master_clear_pending_responses();
        err = can_master_upload_gcode_program(u->node_id, u->slot, data, size, CAN_MASTER_RESPONSE_TIMEOUT_MS);
        free(data);
        if (err != ESP_OK) {
            printf("ERR: relay upload: node=%u slot=%u failed (%s)\n",
                   (unsigned)u->node_id, (unsigned)u->slot, esp_err_to_name(err));
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        can_master_clear_pending_responses();
    }
    return ESP_OK;
}

static esp_err_t relay_execute_cycles(uint32_t cycles)
{
    static const relay_step_t relay_steps[] = {
        {CAN_MASTER_NODE2_ID, RELAY_FORWARD_SLOT, "node2 forward: HOME -> pick A -> place B -> HOME"},
        {CAN_MASTER_NODE1_ID, RELAY_FORWARD_SLOT, "node1 forward: HOME -> pick B -> place C -> HOME"},
        {CAN_MASTER_NODE1_ID, RELAY_RETURN_SLOT,  "node1 return:  pick C -> place B -> HOME"},
        {CAN_MASTER_NODE2_ID, RELAY_RETURN_SLOT,  "node2 return:  pick B -> place A -> HOME"},
    };

    esp_err_t err = relay_upload_programs();
    if (err != ESP_OK) return err;

    for (size_t i = 0; i < CONFIGURED_NODE_COUNT; i++) {
        uint8_t node = CONFIGURED_NODE_IDS[i];
        printf("RELAY: arm + home node=%u\n", (unsigned)node);
        err = relay_arm_and_home_node(node);
        if (err != ESP_OK) return err;
    }

    err = relay_broadcast_home_all();
    if (err != ESP_OK) return err;

    for (uint32_t cycle = 0; cycle < cycles; cycle++) {
        if (relay_stop_requested()) return ESP_ERR_INVALID_STATE;
        relay_set_cycle_progress(cycle + 1U);
        printf("RELAY: cycle %" PRIu32 "/%" PRIu32 "\n", cycle + 1U, cycles);

        for (size_t i = 0; i < ARRAY_SIZE(relay_steps); i++) {
            err = relay_run_step(&relay_steps[i]);
            if (err != ESP_OK) return relay_stop_requested() ? ESP_ERR_INVALID_STATE : err;
        }
    }

    printf("OK: relay orchestrator finished %" PRIu32 " cycle(s)\n", cycles);
    return ESP_OK;
}

static void relay_task(void *arg)
{
    esp_err_t err = relay_execute_cycles((uint32_t)(uintptr_t)arg);
    if (err == ESP_ERR_INVALID_STATE && relay_stop_requested()) {
        printf("RELAY: stopped\n");
    } else if (err != ESP_OK) {
        printf("ERR: relay aborted (%s)\n", esp_err_to_name(err));
    }
    relay_set_state(false, false, 0, 0);
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
    if (argc == 3 && strcmp(argv[2], "status") == 0) return cmd_can_relay_status();
    if (argc == 3 && strcmp(argv[2], "stop") == 0) {
        if (!relay_is_running()) {
            printf("RELAY: idle\n");
            return 0;
        }
        relay_request_stop();
        esp_err_t err = stop_all_flushed(50);
        printf(err == ESP_OK ? "OK: relay stop requested\n" : "ERR: relay stop failed\n");
        return 0;
    }

    uint32_t cycles = 1;
    if (argc == 3 && !parse_u32_arg(argv[2], 1U, 100000U, &cycles)) {
        printf("ERR: invalid cycle count\n");
        return 0;
    }
    if (relay_is_running()) {
        printf("ERR: relay is already running\n");
        return 0;
    }

    relay_set_state(true, false, cycles, 0);
    BaseType_t created = xTaskCreate(relay_task,
                                     "relay_task",
                                     RELAY_TASK_STACK_SIZE,
                                     (void *)(uintptr_t)cycles,
                                     RELAY_TASK_PRIORITY,
                                     NULL);
    if (created != pdPASS) {
        relay_set_state(false, false, 0, 0);
        printf("ERR: relay task start failed\n");
        return 0;
    }

    printf("OK: relay started (%" PRIu32 " cycle(s)); use `can relay status` or `can stop`\n", cycles);
    return 0;
}

static int usage(const char *text)
{
    printf("Usage: %s\n", text);
    return 0;
}

static int cmd_can(int argc, char **argv)
{
    if (argc < 2) {
        printf("%s", CAN_USAGE);
        return 0;
    }

    const char *cmd = argv[1];
    if (strcmp(cmd, "nodes") == 0) return cmd_can_nodes();
    if (strcmp(cmd, "stop") == 0) {
        if (relay_is_running()) relay_request_stop();
        esp_err_t err = stop_all_flushed(50);
        printf(err == ESP_OK
                   ? (relay_is_running() ? "OK: stop sent, relay stop requested\n" : "OK: stop sent\n")
                   : "ERR: stop failed\n");
        return 0;
    }
    if (strcmp(cmd, "relay") == 0) {
        if (argc != 2 && argc != 3) return usage("can relay [cycles]\n       can relay status\n       can relay stop");
        return cmd_can_relay(argc, argv);
    }
    if (relay_is_running()) {
        printf("ERR: relay is running; use `can relay status` or `can stop`\n");
        return 0;
    }

    if (strcmp(cmd, "init") == 0) {
        esp_err_t err = can_master_init();
        printf(err == ESP_OK ? "OK: can init\n" : "ERR: can init failed (%s)\n", esp_err_to_name(err));
        return 0;
    }
    if (strcmp(cmd, "sync") == 0) {
        esp_err_t err = can_master_sync_start_all();
        printf(err == ESP_OK ? "OK: sync start sent\n" : "ERR: sync start failed\n");
        return 0;
    }
    if (strcmp(cmd, "run_sync") == 0) {
        if (argc != 4 && argc != 5) return usage("can run_sync all <path> [slot]");
        return cmd_can_run_sync(argc, argv);
    }
    if (strcmp(cmd, "seq") == 0) {
        if (argc != 2 && argc != 3) return usage("can seq [slot]");
        return cmd_can_seq(argc, argv);
    }
    if (strcmp(cmd, "arm") == 0 || strcmp(cmd, "disarm") == 0 || strcmp(cmd, "home") == 0) {
        if (argc != 3) return usage(strcmp(cmd, "home") == 0 ? "can home <node|all>" : "can <arm|disarm> <node|all>");
        node_response_fn_t fn = strcmp(cmd, "arm") == 0 ? can_master_arm : strcmp(cmd, "home") == 0 ? can_master_home : can_master_disarm;
        return cmd_can_response_like(cmd, argv[2], fn);
    }
    if (strcmp(cmd, "status") == 0) {
        if (argc != 3) return usage("can status <node|all>");
        return cmd_can_status_like(argv[2]);
    }
    if (strcmp(cmd, "sensors") == 0) {
        if (argc != 3) return usage("can sensors <node|all>");
        return cmd_can_sensors_like(argv[2]);
    }
    if (strcmp(cmd, "prepare") == 0 || strcmp(cmd, "run") == 0 || strcmp(cmd, "delete") == 0) {
        if (argc != 4) return usage("can <prepare|run|delete> <node|all> <slot>");
        slot_op_t op = strcmp(cmd, "prepare") == 0 ? SLOT_OP_PREPARE : strcmp(cmd, "run") == 0 ? SLOT_OP_RUN : SLOT_OP_DELETE;
        return cmd_can_slot_like(op, cmd, argv[2], argv[3]);
    }
    if (strcmp(cmd, "upload") == 0) {
        if (argc != 4) return usage("can upload <node|all> <slot>");
        return cmd_can_slot_like(SLOT_OP_UPLOAD, "upload", argv[2], argv[3]);
    }
    if (strcmp(cmd, "upload_file") == 0) {
        if (argc != 5) return usage("can upload_file <node|all> <slot> <path>");
        return cmd_can_upload_file(argv[2], argv[3], argv[4]);
    }

    printf("Unknown can subcommand\n");
    return 0;
}

static void register_commands(void)
{
    esp_console_cmd_t cmd = {.command = "can", .help = "CAN master control", .hint = NULL, .argtable = NULL, .func = &cmd_can};
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void cmd_control_start(void)
{
    esp_console_repl_t *s_repl = NULL;
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
