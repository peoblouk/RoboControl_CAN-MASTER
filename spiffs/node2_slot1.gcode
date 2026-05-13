; ==========================================
; node2_slot1.gcode
; node2: pick B -> place A
; ==========================================
; B = X95 Y190 Z-10 P-50
; via with cube = X150 Y0, lifted above the surface (P-80 keeps this point reachable)
; A = X190 Y0   Z-20 P-50
;
; Fast empty moves: F6000
; Carry moves with cube: F4800
; Final 5 mm approach: F1200

G21
G90

; Open before pickup.
M10
G4 P120

; ---------- B: pick cube ----------
G1 X95 Y190 Z-5  P-50 F6000
G1 X95 Y190 Z-10 P-50 F1200
G4 P80

M11
G4 P260

; Lift above the surface, then carry through the via point.
G1 X95 Y190 Z40 P-50 F1200
G1 X150 Y0 Z40 P-60 F4800

; ---------- A: place cube ----------
G1 X190 Y0 Z-15 P-50 F4800
G1 X190 Y0 Z-20 P-50 F1200
G4 P80

M10
G4 P220

; Small retreat after release.
G1 X190 Y0 Z-15 P-50 F3000

; Return HOME
G0 X115 Y0 Z123 P-60

M30
