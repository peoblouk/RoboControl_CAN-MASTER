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
can probe
can loopback
can search [from] [to]
can nodes
can arm <node|all>
can disarm <node|all>
can home <node|all>
can status <node|all>
can prepare <node|all> <slot>
can run <node|all> <slot>
can delete <node|all> <slot>
can upload <node|all> <slot>   (nahraje DEFAULT_NC)
can stop
can sync
can seq [slot]
```

## Kratke priklady

```text
can search
can arm all
can upload all 0
can prepare all 0
can sync
```

## Kde co nastavit

- SPI piny, MCP2515 oscilator, bitrate, timeouty: `main/can_master/can_master.h`
- Nody pro `all` v konzoli: `main/cmd_control/cmd_control.c` (`CONFIGURED_NODE_IDS`)
- Vychozi NC program pro `can upload`: `main/cmd_control/cmd_control.c` (`DEFAULT_NC`)
