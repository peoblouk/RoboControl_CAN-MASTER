; ==========================================
; node2 slot1
; LEFT robot: HOME -> pick B -> place A -> HOME
; ==========================================
; Starter coordinates for node2.
; The structure matches the proven pick_cube motion profile, but these
; coordinates are intended as a first draft for the left robot and should
; be tuned on the real setup.
;
; B (pickup on node2) - tune as needed:
;   hover    X90 Y0 Z25
;   approach X90 Y0 Z-110
;   pick     X90 Y0 Z-140
; A (place on node2) - tune as needed:
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

; HOME -> B
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

; B -> A
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

; A -> HOME
G0 X0 Y0 Z25 P-20
G4 P150
G0 X0 Y0 Z0 P-53
M30
