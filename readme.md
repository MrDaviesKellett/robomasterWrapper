# robowrap

Student-first wrapper for the RoboMaster EP Python SDK.

The design goal is the same one shown in [test.py](test.py): students should be able to write robot behavior as a sequence of clear intentions such as `move`, `detect`, `grab`, and `close` without learning the SDK transport layer first. Grouped subsystems like `robot.cam`, `robot.arm`, and `robot.gun` mirror the physical robot and keep the first classroom exercise short.

## Project links

- PyPI package: [robowrap on PyPI](https://pypi.org/project/robowrap/)
- Modern SDK package: [robomaster-sdk-modern on PyPI](https://pypi.org/project/robomaster-sdk-modern/)
- Modern SDK repository: [RoboMaster-SDK (maintained fork)](https://github.com/MrDaviesKellett/RoboMaster-SDK)
- Original SDK: [DJI RoboMaster SDK](https://github.com/dji-sdk/RoboMaster-SDK)

## Python support

robowrap now supports modern Python and is packaged for **Python 3.9+**, including **Python 3.14+**.

This change was made because the original DJI RoboMaster SDK appears to be unmaintained and still targets older Python releases. To keep the RoboMaster EP usable on current Python versions, robowrap now depends on the maintained [`robomaster-sdk-modern`](https://pypi.org/project/robomaster-sdk-modern/) package instead.

## Installation

```sh
python -m pip install --upgrade pip
python -m pip install robowrap
```

`robowrap` depends on `robomaster-sdk-modern`, so installing `robowrap` is usually enough. Installing both explicitly makes the SDK choice clear.

`robot.cam.view()` uses PySide6 for live display and does not use OpenCV. This avoids the PyAV/OpenCV duplicate FFmpeg conflict on macOS.

## Snake_case policy

- The public wrapper API is now `snake_case` only.
- Legacy camelCase and mixed-case names still work for one transition release and emit `DeprecationWarning`.
- Those aliases are scheduled for removal in `0.9.0`, not before `2026-09-01`.

## Start here

- [Student guide](docs/student_guide.md)
- [Quickstart](docs/quickstart.md)
- [API reference](docs/api_reference.md)
- [Coverage matrix](docs/coverage_matrix.md)
- [Snake case migration](docs/migration_snake_case.md)
- [Troubleshooting](docs/troubleshooting.md)
- [Contributor checklist](docs/contributor_checklist.md)
- [Changelog](CHANGELOG.md)

## Example

```python
from robowrap import RoboMaster

robot = RoboMaster()
robot.set_leds(0, 128, 255)
robot.forward(0.5)
robot.cam.detect_person()
robot.cam.view()
frame = robot.cam.frame()
robot.arm.move_to(x=0, z=50)
robot.close()
```
