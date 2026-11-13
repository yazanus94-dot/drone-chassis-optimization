# Requirements

## Mission requirements (Step 1)

| ID | Requirement | Value | Source |
|---|---|---|---|
| R1 | Payload mass | 0.50 kg | brief |
| R2 | Hover endurance target | >= 20 min at 5 m/s | brief |
| R3 | Cruise speed | 15 m/s | brief |
| R4 | Thrust-to-weight ratio | >= 2.0 | brief |
| R5 | X-frame footprint | 450 mm | brief |
| R6 | Materials | Al-6061, carbon tube, PETG | brief |
| R7 | Battery | 18650 Li-ion, 5 Ah, 6S, 170 Wh/kg, DoD 100% | brief |
| R8 | Propulsion set | 60 g, max thrust 0.9 kg | brief |
| R9 | Safety factor under landing load (5 g x 1.2) | >= 2.0 | brief |
| R10 | Motor excitation band | 100-300 Hz | brief |
| R11 | Modal constraint | f1 outside band (band avoidance) | adapted, see D2 |

## Sizing results (first pass)

- Arm length: 225 mm (450 mm X-frame); frame mass 0.362 kg (CadQuery cross-check 0.362 kg).
- Total mass: 1.841 kg (payload 0.50 + frame 0.362 + propulsion 0.35 x2 + avionics 0.12 + battery 0.42 approx).
- Thrust per motor: 10.3 N (TWR 2.0 at hover); hover electrical power: 172.7 W (momentum theory, FoM 0.70, eta 0.85).
- Drag at cruise: 5.2 N (validated quadratic law).

## Verification mapping

| Requirement | Verified by |
|---|---|
| R1-R8 | `test_first_pass_sizing_matches_hand_calculations`, sizing sanity tests |
| R9 | `run_structural.py` landing load case, SF in `optimization_summary.json` |
| R10-R11 | modal analysis + band-avoidance constraint in `services.py` |
| R2 (target) | flight-time results: selected design 18.5 min (closed-form) / 18.9 min (PyBaMM); shortfall documented honestly |
