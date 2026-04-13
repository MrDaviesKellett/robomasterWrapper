# Student Guide

This guide starts with the easiest robot actions and builds up step by step.

## 1. Connect and close

Start by connecting to the robot and closing it cleanly at the end.

```python
from robowrap import RoboMaster

robot = RoboMaster()
robot.close()
```

Use `close()` at the end of every program.

## 2. Make the robot move

These are the simplest movement commands.

```python
from robowrap import RoboMaster

robot = RoboMaster()
robot.forward(0.5)
robot.back(0.5)
robot.turn_left(90)
robot.turn_right(90)
robot.close()
```

- Distance is in metres.
- Turn angles are in degrees.

## 3. Change lights and play sounds

Use these to give the robot simple feedback.

```python
from robowrap import RoboMaster

robot = RoboMaster()
robot.set_leds(0, 255, 0)
robot.play_sound(0x101)
robot.close()
```

- `set_leds(r, g, b)` uses red, green, blue values from `0` to `255`.

## 4. Use the camera

Start the camera before using vision features.

```python
from time import sleep

from robowrap import RoboMaster

robot = RoboMaster()
robot.cam.start()
robot.cam.detect_person()
sleep(2)
robot.close()
```

- `robot.cam.detect_person()` looks for a person in view.
- `robot.cam.detect_line()` looks for a coloured line.
- `robot.cam.detect_marker()` looks for a marker.

## 5. Move the arm and gripper

Use the arm to reach and the gripper to hold.

```python
from time import sleep

from robowrap import RoboMaster

robot = RoboMaster()
robot.arm.move_to(x=0, z=50)
robot.gripper.open()
sleep(1)
robot.gripper.close()
robot.close()
```

- Arm values use millimetres.
- Start with small arm movements while learning.

## 6. Put actions together

This is a simple full program.

```python
from time import sleep

from robowrap import RoboMaster

robot = RoboMaster()

robot.set_leds(0, 128, 255)
robot.forward(0.5)
robot.turn_left(90)

robot.cam.start()
robot.cam.detect_person()
sleep(2)

robot.arm.move_to(x=0, z=50)
robot.gripper.close()

robot.close()
```

## 7. What to learn next

- Use `robot.move(x=..., y=...)` for more exact movement.
- Use `robot.cam.follow_line()` for line following.
- Use `robot.gun.move_to()` to control the gimbal.
- Use the [API reference](api_reference.md) when you need more options.
