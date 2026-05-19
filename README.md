# CAN Master

ESP-IDF aplikace pro ESP32-C3 (Stamp-C3U) s MCP2515. Po startu inicializuje NVS, připojí SPIFFS, spustí CAN master a konzoli s příkazem `can`. Na pozadí běží status LED indikující stav uzlů.

## Hardware

| Signál | GPIO |
|--------|------|
| SCK    | 4    |
| MISO   | 5    |
| MOSI   | 6    |
| CS     | 7    |
| INT    | 10   |

- MCP2515 oscilátor: 8 MHz
- CAN bitrate: 500 kbps
- Uzly: ID 0x01, 0x02
- Programové sloty na uzel: 4 (0–3)

## Build / Flash

```bash
idf.py set-target esp32c3
idf.py -p PORT build flash monitor
```

SPIFFS image se sestaví z adresáře `spiffs/`. Soubor `spiffs/example.gcode` je na zařízení dostupný jako `/spiffs/example.gcode`.

## Příkazy

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
can run_sync all <path> [slot]
can delete <node|all> <slot>
can stop
can sync
can measure_sync [duration_ms]
can upload <node|all> <slot>
can upload_file <node|all> <slot> <path>
can seq [slot]
can relay [cycles]
can relay status
can relay stop
```

## Příklady

Základní sekvence:

```text
can nodes
can arm all
can home all
can upload_file all 0 example.gcode
can prepare all 0
can sync
```

Synchronizovaný start programu ze SPIFFS:

```text
can run_sync all example.gcode
can run_sync all example.gcode 1
```

`run_sync` podporuje jen `all`; pokud slot nezadáš, použije se slot `0`.

## Příkaz seq

```text
can seq [slot]
```

Zkratka pro ruční testování. Provede `arm all`, nahraje výchozí G-kód na zadaný slot (výchozí `0`), připraví uzly a odešle `SYNC_START`.

## Příkaz relay

```text
can relay
can relay 3
can relay status
can relay stop
```

Relay orchestrátor běží na pozadí. Na začátku nahraje tyto soubory ze SPIFFS:

```text
node1 slot0 ← node1_slot0.gcode
node2 slot0 ← node2_slot0.gcode
node2 slot1 ← node2_slot1.gcode
node1 slot1 ← node1_slot1.gcode
```

Poté provede `arm + home` pro oba uzly se synchronním broadcastem `HOME` a spustí kroky sekvenčně:

```text
node2 slot0
node1 slot0
node1 slot1
node2 slot1
```

`cycles` určuje počet opakování sekvence. Výchozí hodnota je `1`.

## Příkaz measure_sync

```text
can measure_sync
can measure_sync 10
```

Měření latence synchronizace. Použije slot `3`, nahraje krátký program na oba uzly, připraví je, odešle `SYNC_START` a po uplynutí zadané doby odešle `DISARM`. Výchozí doba je `2 ms`.

## Konfigurace

| Co                          | Soubor                               |
|-----------------------------|--------------------------------------|
| CAN piny, bitrate, ID uzlů, timeouty | `main/can_master/can_master.h`  |
| Konzolové příkazy, relay    | `main/cmd_control/cmd_control.c`     |
| Relay sloty, timeouty       | `main/cmd_control/cmd_control.h`     |
| SPIFFS partition            | `partitions.csv`                     |
