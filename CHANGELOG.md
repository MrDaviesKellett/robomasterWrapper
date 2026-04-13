# Changelog

## 0.8.0 - 2026-04-13

- Migrated the public wrapper API to snake_case across `RoboMaster`, `Gun`, `Arm`, `Gripper`, and `Camera`.
- Added temporary deprecation aliases for the previous camelCase and legacy mixed-case names.
- Expanded EP coverage with student-facing facades for chassis subscriptions, PWM output, battery, servo, sensor, sensor adaptor, armor, UART, AI, camera photo/audio, and arm/gripper subscriptions.
- Added classroom-oriented documentation in `docs/`.

### Deprecations

- CamelCase and legacy aliases introduced before `0.8.0` are deprecated as of `2026-04-13`.
- Those aliases are scheduled for removal in `0.9.0`, not before `2026-09-01`.
