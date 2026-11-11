"""Steering identification in the T1 rig (design 3.9, 4.5, 8.3 P4 'identification tools'; SIM_ACCEPTANCE A18).

The field tests B0-B7 and F1-F5 are manual tests with a person next to the vehicle. Here scripts stand in for the
operator, the T1 rig (steering/t1_rig.py) stands in for the vehicle, and the records are written in the field record
formats. The analysis reads the records only.

  yard.py         calibration yard world (walls, panels and posts; exact ray casting; plant side)
  rig.py          IdentRig(T1Rig): bench mode (the bench tool owns the serial line, bridge off) and drive mode
                  (bridge on, the operator uses the control API), record hooks, host rebuild after calibration
  bench.py        bench tool: ESP protocol V3 lines (INFO, CAL JOG/MARK/SOFT/POL/SAVE, CFG, ARM, C, BENCH, D, STOP)
  instruments.py  SIM stand-ins of the operator's instruments: tape (B1), digital angle gauge (B2, B3, B6)
  recorder.py     record writer: recorder archive layout (metadata.json, motor.jsonl, mapping_source/sensors.jsonl.gz),
                  steering sidecar (orchard.steering.timeline.v1, design 4.5) and bench records
  procedures.py   B0, B1, B2, B3, B4, B5, B6, B7, F1, F2, F3, F4, F5 and the session runner
  analysis.py     estimators from the records only (no truth): steering rate, backlash, zero offset, kappa(delta),
                  minimum PWM, tau; duty -> speed through tools/field_calib (C3)
  a18.py          A18(F): seeds x profile variants, known values from the rig (evaluation only), comparison

Every number comes from placeholder profiles: the results are placeholder-based SIM results (자리값 기반 SIM 결과),
not field validation. Docs: docs/steering/IDENT_KO.md.
"""
