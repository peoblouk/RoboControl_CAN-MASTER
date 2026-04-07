# CAN Master (ESP-IDF + MCP2515)

Jednoduchy CAN master pro ESP32-C3. Po startu inicializuje CAN, spusti konzoli a umozni ovladat roboticke nody pres prikaz `can`.

## Build / flash

```bash
idf.py set-target esp32c3
idf.py -p PORT build flash monitor
```

Po pripojeni monitoru zadavej prikazy do konzole (`>>`).

## Implementovane prikazy

```text
can init
can nodes
can arm <node|all>
can disarm <node|all>
can home <node|all>
can status <node|all>
can sensors <node|all>
can prepare <node|all> <slot>
can run <node|all> <slot>
can delete <node|all> <slot>
can upload <node|all> <slot>   (nahraje DEFAULT_GCODE)
can upload_file <node|all> <slot> <path>   (nahraje /spiffs/<path>)
can stop
can sync
can seq [slot]
```

## Kratke priklady

```text
can arm all
can upload all 0
can prepare all 0
can sync
can upload_file all 0 example.gcode
can sensors all
```

## Kde co nastavit

- SPI piny, MCP2515 oscilator, bitrate, timeouty: `main/can_master/can_master.h`
- Nody pro `all` v konzoli: `main/cmd_control/cmd_control.c` (`CONFIGURED_NODE_IDS`)
- Vychozi G-code program pro `can upload`: `main/cmd_control/cmd_control.c` (`DEFAULT_GCODE`)
- SPIFFS soubory se berou z adresare `spiffs/` (napr. `spiffs/example.gcode`)

## Co vraci status a sensors

- `can status <node|all>` vypise jen `armed/disarmed` a `wcofs`
- `can sensors <node|all>` vypise natozeni senzoru/kloubu v deg

Slave posila tyto hodnoty po `GET_STATUS` pres `CAN_INFO` ramce na `0x780 + node_id`. Master z nich pouziva jen:

- `CAN_INFO_WORK_OFFSET (0x01)` pro `wcofs`
- `CAN_INFO_TCP_META (0x03)` kvuli `value_source`
- `CAN_INFO_VALUES_0_2 (0x10)` a `CAN_INFO_VALUES_3_5 (0x11)` pro hodnoty senzoru
