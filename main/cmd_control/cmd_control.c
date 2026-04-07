#include "cmd_control.h"

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "can_master.h"
#include "esp_console.h"
#include "esp_err.h"
#include "esp_log.h"
#include "linenoise/linenoise.h"
#include "sdkconfig.h"

static const char *TAG = "cmd";
static esp_console_repl_t *s_repl = NULL;
static const uint8_t CONFIGURED_NODE_IDS[] = {CAN_MASTER_NODE1_ID, CAN_MASTER_NODE2_ID};
#define CONFIGURED_NODE_COUNT (sizeof(CONFIGURED_NODE_IDS) / sizeof(CONFIGURED_NODE_IDS[0]))
#define CAN_STATE_FLAG_ARMED (1U << 0)

static const char DEFAULT_GCODE[] =
    "G90\n"
    "G0 X130 Y0 Z80\n"
    "G1 X145 Y0 Z70 F1200\n"
    "G1 X145 Y35 Z70 F1200\n"
    "G1 X130 Y35 Z80 F1200\n"
    "M30\n";

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
        printf("  can delete <node|all> <slot>\n");
        printf("  can stop\n");
        printf("  can sync\n");
        printf("  can upload <node|all> <slot>   (uploads DEFAULT_GCODE)\n");
        printf("  can upload_file <node|all> <slot> <path>   (loads /spiffs/<path>)\n");
        printf("  can seq [slot]\n");
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

    if (strcmp(argv[1], "nodes") == 0) {
        return cmd_can_nodes();
    }

    if (strcmp(argv[1], "stop") == 0) {
        esp_err_t err = can_master_stop_all();
        printf(err == ESP_OK ? "OK: stop sent\n" : "ERR: stop failed\n");
        return 0;
    }

    if (strcmp(argv[1], "sync") == 0) {
        esp_err_t err = can_master_sync_start_all();
        printf(err == ESP_OK ? "OK: sync start sent\n" : "ERR: sync start failed\n");
        return 0;
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
