# Deferred arm subsystem (contract v4 §2 — FROZEN NOW because the drive board
# compiles it; arm enablement must not require reflashing nav firmware).
uint8 id             # 0 = base, 1 = elbow, 2 = gripper (firmware truth; legacy app used 1/2/3 — bridge remaps)
uint16 angle_deg
uint16 step_ms       # ms per degree; 40 = legacy default
