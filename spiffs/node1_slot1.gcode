; ==========================================
; node1 slot1
; RIGHT robot: HOME -> pick C -> place B -> HOME
; ==========================================
; This file reuses the existing proven local pick/place pattern.
; On the right robot:
; - local right-hand pickup point corresponds to global C
; - local left-hand place point corresponds to global B
;
; C (pickup on node1):
;   hover    X90 Y0 Z25
;   approach X90 Y0 Z-110
;   pick     X90 Y0 Z-140
; B (place on node1):
;   hover    X-20 Y-250 Z25
;   approach X-20 Y-250 Z-50
;   place    X-20 Y-250 Z-85
; HOME:
;   X0 Y0 Z0

G21
G90
F700

M10
G4 P250

; HOME -> C
G0 X0 Y0 Z25 P-20
G4 P150
G0 X90 Y0 Z25 P-53
G4 P200
G1 X90 Y0 Z-110 P-53 F700
G4 P150
G1 X90 Y0 Z-140 P-53 F450
G4 P250
M11
G4 P350
G1 X90 Y0 Z-110 P-53 F500
G1 X90 Y0 Z25 P-53 F700
G4 P200

; C -> B
G0 X-20 Y-250 Z25 P-20
G4 P200
G1 X-20 Y-250 Z-50 P-30 F700
G4 P150
G1 X-20 Y-250 Z-85 P-53 F450
G4 P250
M10
G4 P300
G1 X-20 Y-250 Z-50 P-30 F500
G1 X-20 Y-250 Z25 P-20 F700
G4 P200

; B -> HOME
G0 X0 Y0 Z25 P-20
G4 P150
G0 X0 Y0 Z0 P-53
M30
