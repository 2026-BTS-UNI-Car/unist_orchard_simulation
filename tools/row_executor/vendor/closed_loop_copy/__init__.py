"""Headless, pure-Python closed loop for encoderless row following (SIM only).

plant (field_response.FieldPlant) -> LiDAR pose -> orchard_perception
raycast_scan (+noise, branches, occlusion, latency, dropouts) and raw-IMU-like
gyro -> sensor-only estimator -> controller -> integer duty -> MCU ramp.

Truth pose/scene geometry is used only to synthesize sensors and to score
metrics. Nothing here is a real-vehicle validation. Requires ``sim/ubts`` on
``sys.path`` (flat test layout): run tests from the repository root with
``python -m pytest sim/ubts/test_closed_loop_*.py`` and the matrix with
``cd sim/ubts && python -m closed_loop.evaluate``.
"""
