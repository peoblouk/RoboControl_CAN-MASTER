; ==========================================
; Simulace "pick & place" bez gripper cmd
; ==========================================
G21
G90

; Prepocet z puvodnich BASE bodu na WORK pri wcofs X=186 Y=0 Z=105.7
; WORK = BASE - WCOFS

; Bezpecna pozice
G1 X34 Y0 Z34.3 P20 F1000
G4 P300

; Nad "pickup"
G1 X14 Y0 Z14.3 P20
G4 P300

; Sestup kousek nad zem (uprav si Z podle reality)
G1 X14 Y0 Z-25.7 P20
G4 P800    ; simulace uchopu

; Zvednout
G1 X14 Y0 Z14.3 P20
G4 P200

; Presun nad "place"
G1 X54 Y0 Z14.3 P20
G4 P300

; Sestup
G1 X54 Y0 Z-25.7 P20
G4 P800    ; simulace pusteni

; Zvednout a konec
G1 X54 Y0 Z14.3 P20
G1 X34 Y0 Z34.3 P20
M30
