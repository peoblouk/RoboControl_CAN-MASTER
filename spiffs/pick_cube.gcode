; ==========================================
; Cyklus A -> B -> HOME -> B -> A -> HOME
; ==========================================
; A (nabrani):
;   hover   = X-20 Y-250 Z25
;   approach= X-20 Y-250 Z-50
;   pick    = X-20 Y-250 Z-85
;
; B (odlozeni):
;   hover   = X90 Y0 Z25
;   approach= X90 Y0 Z-110
;   place   = X90 Y0 Z-140
;
; HOME:
;   X0 Y0 Z0
;
; Pitch:
; - presne A hover s P=-53 je mimo dosah aktualni IK
; - proto hover/travel jede s mirnejsim pitch
; - P=-53 se pouziva az pri spodnim bodu

G21
G90
F700

; A hover/app/pick
; B hover/app/place
; HOME = X0 Y0 Z0
; TRAVEL_PITCH   = -20
; APPROACH_PITCH = -30
; PICK_PITCH     = -53
; APPROACH_FEED  = 700 mm/min
; TOUCH_FEED     = 450 mm/min
; LIFT_FEED      = 500 mm/min

M10
G4 P250

; ---------- A -> PICK ----------
G0 X0 Y0 Z25 P-20
G4 P200
G0 X-20 Y-250 Z25 P-20
G4 P200
G1 X-20 Y-250 Z-50 P-30 F700
G4 P150
G1 X-20 Y-250 Z-85 P-53 F450
G4 P250
M11
G4 P350
G1 X-20 Y-250 Z-50 P-30 F500
G1 X-20 Y-250 Z25 P-20 F700
G4 P200

; ---------- A -> B ----------
G0 X90 Y0 Z25 P-53
G4 P200
G1 X90 Y0 Z-110 P-53 F700
G4 P150
G1 X90 Y0 Z-140 P-53 F450
G4 P250
M10
G4 P300
G1 X90 Y0 Z-110 P-53 F500
G1 X90 Y0 Z25 P-53 F700
G4 P200

; ---------- B -> HOME ----------
G0 X0 Y0 Z25 P-53
G4 P150
G0 X0 Y0 Z0 P-53
G4 P400

; ---------- HOME -> B ----------
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

; ---------- B -> A ----------
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

; ---------- A -> HOME ----------
G0 X0 Y0 Z25 P-20
G4 P150
G0 X0 Y0 Z0 P-53
M30
