# Fixtures

Golden values carried over from the console (`~/makermods-console/web`), which produced them from the Python
originals.

| File | Console source |
|---|---|
| `autocal-golden.json` | `test/so101-autocal.test.ts`, `PY_LIMITS` and `PY_WRAP`: MakerLab's `FeetechCalibrationMixin._compute_mid_and_range_from_limits` and `auto_calibrate_script._nearest_wrap_target`, run under Python |
| `calibrations.json` | `follower_left`: `test/so101.test.ts` `LEFT_FILE`; `autocal_left`, `autocal_right`: `test/so101-autocal.test.ts` `LEFT`, `RIGHT` (real arms' LeRobot calibrations) |
| `so101-trajectory-ik.json` | `test/fixtures/so101-trajectory-ik.json` (copied unchanged): dreamlabs 0.0.30 `so101_faas` forward kinematics and trajectory IK, from the console's `diagnostics/so101_trajectory_ik.py` |
