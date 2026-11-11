"""Native (C++11) steering actuator and potentiometer model with its lockstep driver.

The C++ sources here build one program (decision R1, design 2.3.1, 3.3, 3.4) that speaks the
lockstep protocol v1 (docs/steering/LOCKSTEP_PROTOCOL_KO.md). `build` compiles it on demand;
`steering.actuator_link.NativeSteeringBackend` runs it behind the SteeringBackend contract.
No MCU firmware logic yet (phase P3). Build outputs never go into git (see build.py).
"""

__all__ = ['build']
