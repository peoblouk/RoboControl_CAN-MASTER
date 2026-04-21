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
can relay [cycles]
can relay status
can relay stop
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

## Relay orchestrator

Pro sekvencni predavani kostky mezi dvema roboty je prakticke pouzit:

- lokalni G-code na kazdem uzlu jen pro jeho vlastni pohyby
- nad tim centralni master orchestrator pres CAN

Prikaz:

```text
can relay [cycles]
```

udela tuto sekvenci:

1. `arm` node 1
2. `arm` node 2
3. posle broadcast `HOME` na oba nody, takze reference/home startuje soucasne
4. ceka, az se oba nody vrati do `READY`
5. spusti `PROGRAM_RUN` po jednotlivych krocich:
   - `node2 slot0`: `HOME -> pick A -> place B -> HOME`
   - `node1 slot0`: `HOME -> pick B -> place C -> HOME`
   - `node1 slot1`: `pick C -> place B -> HOME`
   - `node2 slot1`: `pick B -> place A -> HOME`

Rozlozeni po oprave:

- `node2` = levy robot = body `A <-> B`
- `node1` = pravy robot = body `B <-> C`

Tohle je sekvencni orchestrator. Nepouziva `PREPARE + SYNC_START`, protoze pro predavani kostky je tady spravne spoustet jednotlive programy az po dokonceni predchoziho kroku.

Relay bezi na pozadi, takze konzole zustava funkcni. Za behu pouzij:

```text
can relay status
can relay stop
```

Pri cekani na dokonceni kroku relay nepouziva opakovane `GET_STATUS`; cte jen posledni periodicky posilany status z cache masteru.

Typicky postup:

```text
can upload_file 1 0 node1_slot0.gcode
can upload_file 2 0 node2_slot0.gcode
can upload_file 2 1 node2_slot1.gcode
can upload_file 1 1 node1_slot1.gcode
can relay
```

Pro opakovani ukazky vic krat za sebou muzes pouzit treba:

```text
can relay 3
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
