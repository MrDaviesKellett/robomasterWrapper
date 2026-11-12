# Changelog

## Unreleased

- Print the connection route immediately and progress heartbeats every ten seconds; add an optional overall `connect_timeout` (seconds) while preserving the existing connection troubleshooting message. Late SDK connections are closed after timeout.
- Reject `move()` when all movement coordinates are omitted, with accepted arguments and an explicit-zero example; keep `move(x=0, y=0)` supported.
- Standardize validator type errors to name the parameter and accepted Python types.
- Add actionable `send()` guidance to the UART receive error.
- Lead the README with the Python 3.14+ installation requirement.

## 0.9.0 - 2026-11-12

### BREAKING CHANGES

- The 36 expired camelCase and mixed-case aliases have been removed. Use snake_case names now; see `docs/migration_snake_case.md`.
- Push commands for continuous chassis speed/rotation and the blaster LED now return `None` (fire and forget), not a delivery status.
- The `pulse` LED effect now maps to the supported SDK `breath` effect.
- Blaster fire count is restricted to 1–5.
- `continuous_drive` and rotation axis signs follow DJI convention: x forward, y right, continuous z clockwise-positive; `rotate_left` sends negative z and `rotate_right` positive z. Action angles remain counter-clockwise-positive.
- Connection defaults are normalized to `conn_type="ap"`, `protocol="udp"`; SDK connection names are primary, legacy connection aliases remain supported, and unknown names raise `ValueError`.
- UART `subscribe()` raises `NotImplementedError` until upstream receive handling is fixed.

### Fixes by audit finding

- RW-01: round recenter speeds to integers before SDK packing.
- RW-02: blocking actions check failed state as well as wait completion.
- RW-03: left/right continuous rotation and counter-clockwise circles follow DJI examples; hardware capture pending.
- RW-04: document continuous axes x forward, y right, z clockwise, distinct from action angle signs.
- RW-05: remove the circle duration multiplier of 1.5.
- RW-06: map pulse LEDs to the supported breath effect.
- RW-07: return None for unacknowledged speed and blaster LED pushes; document stop/circle results.
- RW-08: validate fire count 1–5.
- RW-09: default to ap/udp, support SDK connection names and legacy aliases, reject unknown names.
- RW-10: avoid duplicate initialization reset; retain auto_reset argument for compatibility.
- RW-11: remove 36 expired legacy aliases and stale warnings; add the missing 0.8.8 entry.
- RW-12: document the two-joint endpoint teaching surface, validate arm travel, and fix fixed-origin wording and pickup/drop positions.
- RW-13: make sample arm targets reachable and within wrapper bounds.
- RW-14: explain default chassis mode ignores yaw and how free/gun mode enables aiming.
- RW-15: convert UART bytes to bytearray and reject unsafe upstream receive subscriptions.
- RW-16: correct the historical positive-Y statement.
- RW-17: remove media queue backpressure during shutdown and restore bounds, preventing stop/close deadlocks.

Breaking changes are included in 0.9.0 per the maintainer’s decision.

## 0.8.8

- Audited baseline release; no separate release notes were recorded.


## 0.8.7 - 2026-06-08

- Fixed camera vision subscription helpers to pass the RoboMaster SDK's own detection constants, avoiding SDK parameter errors for `detect_person`, `detect_line`, `detect_marker`, and `detect_robot`.
- Corrected package metadata so normal installs resolve against `robomaster-sdk-modern>=0.1.1.76`.
- Updated the Python requirement to 3.14+ to match the modern RoboMaster SDK runtime guard.
- Added a no-motion camera hardware check script for connection, frame capture, audio, and vision smoke testing.

## 0.8.6 - 2026-06-08

- Renamed chassis continuous speed control to `continuous_drive(...)`.
- Changed ambiguous `set_speed(...)` and legacy `setSpeed(...)` to raise a clear error pointing students to `set_default_speed(...)` or `continuous_drive(...)`.
- Added `set_default_speed(...)` for distance move defaults while keeping named `speed` and `turn_speed` arguments as per-command overrides.
- Fixed left/right distance movement to match the RoboMaster SDK coordinate system: SDK `move` y is positive to the right; `left()` sends negative y.
- Corrected chassis rear wheel RPM mapping to SDK wheel order.

## 0.8.5 - 2026-04-23

- Fixed strafing direction so `left()` and `right()` map correctly to chassis motion.
- Hardened `gun.recenter()` with a default timeout and a `moveto(0, 0)` fallback when completion is not reported.
- Added `cam.show()` as a compatibility alias to `cam.view()`.
- Added camera stream self-recovery in `cam.frame()` after repeated decode/read failures.
- Added connection retry controls (`connect_retries`, `connect_retry_delay`) to improve reliability on unstable links.

## 0.8.4 - 2026-04-15

- Updated the minimum SDK dependency to `robomaster-sdk-modern>=1.1.1.76`.
- Replaced OpenCV display with a PySide6 live viewer to avoid PyAV/OpenCV duplicate FFmpeg conflicts on macOS.
- Changed `Camera.view()` to start a non-blocking live viewer process.
- Added `Camera.frame()` for code that needs the latest camera frame.
- Added `Camera.stop_view()` to close the viewer from code.
- Updated camera troubleshooting docs for the PyAV/PySide6 viewer path.

## 0.8.3 - 2026-04-15

- Updated the minimum SDK dependency to `robomaster-sdk-modern>=1.1.1.76`.
- Removed OpenCV from default dependencies because the modern SDK uses PyAV and both packages can load conflicting FFmpeg libraries on macOS.
- Changed `Camera.view()` to return frames without importing OpenCV by default.
- Added optional OpenCV display support behind `Camera.view(show=True, allow_unsafe_opencv=True)` and the `viewer` extra.
- Updated camera troubleshooting docs for the PyAV/OpenCV duplicate-library warning.

## 0.8.2 - 2026-04-13

- Updated packaging to target modern Python releases with `requires-python >=3.9`, including Python 3.14+.
- Switched the wrapper dependency from DJI's legacy `robomaster` package to `robomaster-sdk-modern>=0.1.1.69`.
- Removed the legacy Windows-only `opencv-python==4.2.0.34` pin and now depend on current `opencv-python`.
- Updated installation and support docs to explain the move to the maintained modern SDK and why the change was needed.
- Updated the PyPI publishing workflow to build on Python 3.14.

## 0.8.0 - 2026-04-13

- Migrated the public wrapper API to snake_case across `RoboMaster`, `Gun`, `Arm`, `Gripper`, and `Camera`.
- Added temporary deprecation aliases for the previous camelCase and legacy mixed-case names.
- Expanded EP coverage with student-facing facades for chassis subscriptions, PWM output, battery, servo, sensor, sensor adaptor, armor, UART, AI, camera photo/audio, and arm/gripper subscriptions.
- Added classroom-oriented documentation in `docs/`.

### Deprecations

- CamelCase and legacy aliases introduced before `0.8.0` are deprecated as of `2026-04-13`.
- The original removal target was `0.9.0`, not before `2026-09-01`; removal is implemented in 0.9.0.
