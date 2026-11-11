"""Safety scripts of the front-steer executor-in-the-loop F-SIM (design 8.3 P4 'safety scripts'; SIM_ACCEPTANCE A17, A19,
A22 and the P4 checks A5, A8, A11, A13 F).

  catalogue   the scripts: each one an episode spec of tools/row_executor/sim_in_loop_steer.py (world, controller,
              rig faults, operator / client actions) and its stated expected outcome
  measure     measurements from one episode's artefacts (rig serial log, MCU lines and channels, trace, host latch log,
              executor events) and the spec 6.7 judgement-tick recomputation from the MCU channels
  acceptance  runs the catalogue and assembles the check tables (A5, A8, A11, A13, A17, A19, A22) with pass / fail

Evaluation code (A13 role 'evaluator'): it reads the plant, the MCU channels and the trace truth columns; nothing here
feeds the executor. Every number is a placeholder-based SIM result (자리값 기반 SIM 결과), not field validation.
"""
