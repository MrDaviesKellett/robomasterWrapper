# Changelog

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
- Those aliases are scheduled for removal in `0.9.0`, not before `2026-09-01`.
