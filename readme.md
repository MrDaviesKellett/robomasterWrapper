# robowrap

Student-first wrapper for the DJI RoboMaster EP Python SDK.

The design goal is the same one shown in [test.py](test.py): students should be able to write robot behavior as a sequence of clear intentions such as `move`, `detect`, `grab`, and `close` without learning the SDK transport layer first. Grouped subsystems like `robot.cam`, `robot.arm`, and `robot.gun` mirror the physical robot and keep the first classroom exercise short.

## Project links

- PyPI package: [robowrap on PyPI](https://pypi.org/project/robowrap/)
- Original SDK: [DJI RoboMaster SDK](https://github.com/dji-sdk/RoboMaster-SDK)

## Python support

The upstream RoboMaster SDK still targets older Python releases. Use **Python 3.7** or **Python 3.8** for real robot control.

## Installation

```sh
python3.8 -m pip install --upgrade pip
python3.8 -m pip install opencv-python simple_pid robomaster --user
python3.8 -m pip install robowrap --user
```

To build and publish a new release:

```sh
python3 -m pip install --upgrade build twine
python3 -m build
python3 -m twine upload dist/*
```

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
robot.arm.move_to(x=0, z=50)
robot.close()
```
