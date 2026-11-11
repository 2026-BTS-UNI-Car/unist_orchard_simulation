"""MCU conformance scripts and their runner (ESP protocol V3, design 8.3 P3, acceptance A19 MCU part and A20).

  scripts/*.json      conformance scripts, schema ubts.steer_conformance.v1
                      (docs/steering/ESP_PROTOCOL_V3_DRAFT_KO.md section 13); expected values are set by
                      hand from the specification, never from another implementation (decision R1)
  run_conformance     loads and validates the scripts, runs them against the native firmware build
                      through the lockstep v2 firmware mode (docs/steering/LOCKSTEP_PROTOCOL_KO.md
                      section 11) and compares the MCU lines, pins and channel changes

All values are placeholders. Passing these scripts is a placeholder-based SIM result, not field validation.
"""

__all__ = ['run_conformance']
