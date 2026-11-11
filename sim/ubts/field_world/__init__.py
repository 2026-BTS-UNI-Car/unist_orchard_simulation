"""Field worlds of the steering SIM (design 5.1 W2 / W3, 5.2, 8.3 P7).

One field world specification (scene_spec.FieldWorldSpec) gives both a raster copy (the F-SIM sensing and contact
model, RASTER_V1 classes) and a Gazebo SDF (build_field_world). Modules:

  scene_spec         the specification (canonical JSON, SHA-256, sealed-corridor refusal)
  raster_ops         pure-numpy helpers: components, merged rectangles, supercover DDA, the raster-local <-> W
                     transform, number formatting
  build_field_world  raster copy + SDF + manifest + eval_truth (modes R and T)
  coord_lint         A16 lint (lat/lon-like number pairs, map YAML origins, GNSS path words)
  orchard_stats      10/10 development-corridor statistics for the procedural orchard (W3)
  real_scan_compare  A6: synthetic scans against the recorded scans at SLAM trajectory nodes
  a7_equivalence     A7: mode R + Embree + clip node against the raster DDA
  a12_twin           A12: F-SIM against G-SIM with the same open-loop scripts

Worlds made from the 10/10 maps are 'the orchard as SLAM saw it', not physical truth. Placeholder-based scenes give
placeholder-based SIM results (자리값 기반 SIM 결과), not field validation.
"""
