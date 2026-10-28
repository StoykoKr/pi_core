# Per-motor electrics from the VESCs (contract v4 §2 — FROZEN).
# Battery volts are display-only; nothing automated consumes this.
float32 volts_left
float32 volts_right
float32 amps_left
float32 amps_right
uint8 fault          # 0 = healthy; bitmask reserved for VESC fault codes
