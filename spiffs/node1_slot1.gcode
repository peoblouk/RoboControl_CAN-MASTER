; ==========================================
; node1_slot1.gcode
; node1: pick C -> place B
; ==========================================
; C = X185 Y0    Z-20 P-50
; via with cube = X150 Y0 Z50 (P-80 keeps this point reachable)
; B = X110 Y-225 Z-15 P-40
;
; Fast empty moves: F6000
; Carry moves with cube: F4800
; Final 5 mm approach: F1200

G21
G90

; Open before pickup.
M10
G4 P120

; ---------- C: pick cube ----------
G1 X182 Y0 Z-15 P-50 F6000
G1 X182 Y0 Z-20 P-50 F1200
G4 P80

M11
G4 P260

; Lift above the surface, then carry back through the via point.
G1 X185 Y0 Z-0 P-50 F1200
G1 X150 Y0 Z50 P-60 F4800
G1 X110 Y-220 Z40 P-40 F4800

; ---------- B: place cube ----------
G1 X110 Y-225 Z10 P-40 F4800
G1 X110 Y-225 Z-15 P-40 F1200
G4 P80

M10
G4 P220

; Small retreat after release.
G1 X110 Y-220 Z40 P-40 F3000

; Return HOME
G0 X115 Y0 Z123 P-60

M30
