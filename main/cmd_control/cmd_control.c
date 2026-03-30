#include "cmd_control.h"

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
#define CAN_SEARCH_TIMEOUT_MS 80

static const char DEFAULT_NC[] =
    "G90\n"
    "G0 X130 Y0 Z80\n"
    "G1 X145 Y0 Z70 F1200\n"
    "G1 X145 Y35 Z70 F1200\n"
    "G1 X130 Y35 Z80 F1200\n"
    "M30\n";

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
    if (end == arg || *end != '\0' || parsed > CAN_NODE_ID_MAX) {
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

static void print_data_bytes(const uint8_t *data, uint8_t len)
{
    printf("[");
    for (uint8_t i = 0; i < len; i++) {
        printf("%s%02X", (i == 0) ? "" : " ", (unsigned)data[i]);
    }
    printf("]");
}

static int cmd_can_nodes(void)
{
    print_configured_nodes();
    return 0;
}

static int cmd_can_probe(void)
{
    if (!can_master_is_ready()) {
        esp_err_t init_err = can_master_init();
        if (init_err != ESP_OK) {
            printf("WARN: CAN not ready and init retry failed (%s)\n", esp_err_to_name(init_err));
        }
    }

    can_master_probe_t probe = {0};
    esp_err_t err = can_master_debug_probe(&probe);
    if (err != ESP_OK) {
        printf("ERR: probe failed (%s)\n", esp_err_to_name(err));
        printf("Hint: check SPI wiring/ground and MCP2515 oscillator setting (CAN_MASTER_MCP2515_OSC_HZ).\n");
        return 0;
    }

    printf("CANSTAT=0x%02X CANCTRL=0x%02X CNF1=0x%02X CNF2=0x%02X CNF3=0x%02X\n",
           (unsigned)probe.canstat,
           (unsigned)probe.canctrl,
           (unsigned)probe.cnf1,
           (unsigned)probe.cnf2,
           (unsigned)probe.cnf3);
    printf("CANINTE=0x%02X CANINTF=0x%02X EFLG=0x%02X TXB0CTRL=0x%02X RXB0CTRL=0x%02X RXB1CTRL=0x%02X\n",
           (unsigned)probe.caninte,
           (unsigned)probe.canintf,
           (unsigned)probe.eflg,
           (unsigned)probe.txb0ctrl,
           (unsigned)probe.rxb0ctrl,
           (unsigned)probe.rxb1ctrl);
    return 0;
}

static int cmd_can_loopback(void)
{
    can_master_loopback_result_t result = {0};
    esp_err_t err = can_master_debug_loopback(&result);

    printf("TX id=0x%03X dlc=%u data=", (unsigned)result.tx_id, (unsigned)result.dlc);
    print_data_bytes(result.tx_data, result.dlc);
    printf("\n");
    printf("RX id=0x%03X dlc=%u data=", (unsigned)result.rx_id, (unsigned)result.dlc);
    print_data_bytes(result.rx_data, result.dlc);
    printf("\n");

    if (err == ESP_OK && result.matched) {
        printf("OK: MCP2515 loopback matched\n");
    } else {
        printf("ERR: loopback failed (%s)\n", esp_err_to_name(err));
    }
    return 0;
}

static int cmd_can_search(int argc, char **argv)
{
    uint8_t node_from = 1;
    uint8_t node_to = CAN_NODE_ID_MAX;

    if (argc == 3 || argc == 4) {
        char *end = NULL;
        unsigned long parsed_from = strtoul(argv[2], &end, 0);
        if (end == argv[2] || *end != '\0' || parsed_from == 0 || parsed_from > CAN_NODE_ID_MAX) {
            printf("ERR: invalid start node\n");
            return 0;
        }
        node_from = (uint8_t)parsed_from;
        node_to = node_from;
    }

    if (argc == 4) {
        char *end = NULL;
        unsigned long parsed_to = strtoul(argv[3], &end, 0);
        if (end == argv[3] || *end != '\0' || parsed_to == 0 || parsed_to > CAN_NODE_ID_MAX) {
            printf("ERR: invalid end node\n");
            return 0;
        }
        node_to = (uint8_t)parsed_to;
    }

    if (node_from > node_to) {
        printf("ERR: invalid range\n");
        return 0;
    }

    printf("Scanning nodes %u..%u (timeout %u ms)\n",
           (unsigned)node_from,
           (unsigned)node_to,
           (unsigned)CAN_SEARCH_TIMEOUT_MS);

    size_t found_count = 0;
    for (uint8_t node = node_from; node <= node_to; node++) {
        can_master_response_t resp = {0};
        esp_err_t err = can_master_request_status(node, &resp, CAN_SEARCH_TIMEOUT_MS);
        if (err == ESP_OK) {
            print_response_line(&resp);
            found_count++;
        }
        if (node == CAN_NODE_ID_MAX) {
            break;
        }
    }

    printf("Search complete: found %u node(s)\n", (unsigned)found_count);
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
                print_response_line(&resp);
            } else {
                printf("ERR: node=%u status failed\n", (unsigned)CONFIGURED_NODE_IDS[i]);
            }
        }
        return 0;
    }

    can_master_response_t resp = {0};
    esp_err_t err = can_master_request_status(node, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    if (err == ESP_OK) {
        print_response_line(&resp);
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

    char *end = NULL;
    unsigned long parsed_slot = strtoul(slot_arg, &end, 0);
    if (end == slot_arg || *end != '\0' || parsed_slot >= CAN_PROGRAM_SLOT_COUNT) {
        printf("ERR: invalid slot\n");
        return 0;
    }
    const uint8_t slot = (uint8_t)parsed_slot;
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
                err = can_master_upload_nc_program(current_node,
                                                   slot,
                                                   (const uint8_t *)DEFAULT_NC,
                                                   strlen(DEFAULT_NC),
                                                   CAN_MASTER_RESPONSE_TIMEOUT_MS);
            } else if (do_delete) {
                err = can_master_program_delete(current_node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
            }

            if (err == ESP_OK) {
                if (do_upload) {
                    printf("OK: uploaded default NC to node=%u slot=%u\n",
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
        err = can_master_upload_nc_program(node,
                                           slot,
                                           (const uint8_t *)DEFAULT_NC,
                                           strlen(DEFAULT_NC),
                                           CAN_MASTER_RESPONSE_TIMEOUT_MS);
    } else if (do_delete) {
        err = can_master_program_delete(node, slot, &resp, CAN_MASTER_RESPONSE_TIMEOUT_MS);
    }

    if (err == ESP_OK) {
        if (do_upload) {
            printf("OK: uploaded default NC to node=%u slot=%u\n", (unsigned)node, (unsigned)slot);
        } else {
            print_response_line(&resp);
        }
    } else {
        printf("ERR: %s failed\n", op);
    }
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
        printf("  can probe\n");
        printf("  can loopback\n");
        printf("  can search [from] [to]\n");
        printf("  can nodes\n");
        printf("  can arm <node|all>\n");
        printf("  can disarm <node|all>\n");
        printf("  can home <node|all>\n");
        printf("  can status <node|all>\n");
        printf("  can prepare <node|all> <slot>\n");
        printf("  can run <node|all> <slot>\n");
        printf("  can delete <node|all> <slot>\n");
        printf("  can stop\n");
        printf("  can sync\n");
        printf("  can upload <node|all> <slot>   (uploads DEFAULT_NC)\n");
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

    if (strcmp(argv[1], "probe") == 0) {
        return cmd_can_probe();
    }

    if (strcmp(argv[1], "loopback") == 0) {
        return cmd_can_loopback();
    }

    if (strcmp(argv[1], "search") == 0) {
        if (argc != 2 && argc != 3 && argc != 4) {
            printf("Usage: can search [from] [to]\n");
            return 0;
        }
        return cmd_can_search(argc, argv);
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
