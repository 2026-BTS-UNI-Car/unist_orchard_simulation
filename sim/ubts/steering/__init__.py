"""Shared core of the front-wheel steering SIM (design docs/steering/DESIGN_v1_KO.md).

Submodules (imported explicitly by callers; nothing is imported here):

  vehicle_profile  vehicle profile YAML loader (schema ubts.vehicle_profile.v2)
  geometry         bicycle angle <-> wheel angles, limits, sweep, footprint
  drive            drive curves, stop modes, G-SIM DC motor line values
  backend_api      steered plant <-> steering actuator backend contract and wire codec
  l0_servo         Python L0 ideal steering servo backend
  actuator_link    NativeSteeringBackend: lockstep link to the C++ actuator model
  native           C++ steering actuator and potentiometer model, build.py
  steered_plant    SteeredPlant fast-SIM plant, SteeringActuation, plant factory

All values are SIM placeholders unless the profile says otherwise. Results built on
them are placeholder-based SIM results, not field validation.
"""

__all__ = ['vehicle_profile', 'geometry', 'drive', 'backend_api',
           'l0_servo', 'actuator_link', 'native', 'steered_plant']
