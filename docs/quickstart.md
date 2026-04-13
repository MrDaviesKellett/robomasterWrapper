# Quickstart

## Why this wrapper works well for students

The current usage style is effective because students can write robot behavior as a sequence of clear intentions such as `move`, `detect`, `grab`, and `close` without first learning SDK internals. Grouped subsystems like `robot.cam`, `robot.arm`, and `robot.gun` mirror how students think about the physical robot, not how the transport or protocol stack is built. That lowers onboarding friction and keeps classroom feedback loops short.

As coverage expands, the same rule applies everywhere in robowrap:

- beginner-safe defaults
- plain-language method names
- explicit units and ranges
- predictable behavior across modules
- advanced options only when needed

## First robot interaction

1. Power on the RoboMaster EP and connect it in `device` or `usb` mode.
2. Install Python 3.8 and the upstream RoboMaster SDK.
3. Run the example below.

```python
from time import sleep

from robowrap import RoboMaster

robot = RoboMaster()

robot.set_leds(0, 200, 255)
robot.forward(0.5)
robot.turn_left(90)

robot.cam.start()
robot.cam.detect_person()
sleep(2)

robot.arm.move_to(x=0, z=50)
robot.gripper.open()
sleep(1)
robot.gripper.close()

robot.close()
```

## What each line is doing

- `RoboMaster()` connects to the EP, builds the subsystem wrappers, and switches to chassis-led mode by default.
- `set_leds`, `forward`, and `turn_left` are top-level chassis or robot actions.
- `robot.cam` controls the camera and on-device vision.
- `robot.arm` and `robot.gripper` control the manipulator.
- `close()` stops motion, shuts down streams, and closes the SDK connection cleanly.

## Safe defaults

- Motion helpers block by default for position-based actions such as `move`, `forward`, `turn_left`, `gun.move_to`, and `arm.move_to`.
- Streaming and subscriptions do not block.
- Most invalid parameter values raise `ValueError` immediately with a clear range message instead of silently clamping.

## Next steps

- Learn the full surface in [api_reference.md](api_reference.md)
- Check old-to-new naming in [migration_snake_case.md](migration_snake_case.md)
- Use [troubleshooting.md](troubleshooting.md) if connection, stream, or vision setup fails
