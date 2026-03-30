# CAN MAIN Node (ESP-IDF / MCP2515 SPI)

This project is now configured as a CAN MAIN node that:
1. Initializes MCP2515 over SPI on the MCU.
2. Sends `ARM` to all configured robot nodes.
3. Uploads NC/G-code to a slot via `UPLOAD_BEGIN/DATA/END`.
4. Sends `PREPARE` to all robot nodes.
5. Starts all robots in sync using broadcast `SYNC_START`.
6. Periodically polls `GET_STATUS` from each node.

## Where to configure

- Robot node IDs: `main/led_strip_rmt_ws2812_main.c` (`ROBOT_NODE_IDS`)
- NC program text: `main/led_strip_rmt_ws2812_main.c` (`NC_PROGRAM`)
- Program slot: `main/led_strip_rmt_ws2812_main.c` (`PROGRAM_SLOT`)
- SPI pins, MCP2515 oscillator, CAN bitrate: `main/can_master.h`

## Build and flash

```bash
idf.py set-target esp32c3
idf.py -p PORT build flash monitor
```
