# Snake Case Migration

## Policy

- `snake_case` is now the only supported primary public API style.
- Legacy camelCase and mixed-case names still work in `0.8.x`.
- Every legacy alias emits `DeprecationWarning`.
- Removal target: `0.9.0`, not before `2026-09-01`.

## Top-level `RoboMaster`

| Old name | New name |
| --- | --- |
| `getModule` | `get_module` |
| `setRobotMode` | `set_robot_mode` |
| `getRobotMode` | `get_robot_mode` |
| `getSN` | `get_sn` |
| `getVersion` | `get_version` |
| `playAudio` | `play_audio` |
| `playSound` | `play_sound` |
| `setLEDs` | `set_leds` |
| `stopAfter` | `stop_after` |
| `setSpeed` | `set_speed` |
| `rotateLeft` | `rotate_left` |
| `rotateRight` | `rotate_right` |
| `turnLeft` | `turn_left` |
| `turnRight` | `turn_right` |
| `setWheelRPMs` | `set_wheel_rpms` |

## `Gun`

| Old name | New name |
| --- | --- |
| `setLEDs` | `set_leds` |
| `setLED` | `set_led` |
| `moveto` | `move_to` |

## `Arm`

| Old name | New name |
| --- | --- |
| `moveTo` | `move_to` |

## `Camera`

| Old name | New name |
| --- | --- |
| `setResolution` | `set_resolution` |
| `stopDetect` | `stop_detect` |
| `setPID` | `set_pid` |
| `setDetectMode` | `set_detect_mode` |
| `setVisionDebug` | `set_vision_debug` |
| `setDebugColor` | `set_debug_color` |
| `detectPerson` | `detect_person` |
| `detectGesture` | `detect_gesture` |
| `detectLine` | `detect_line` |
| `detectMarker` | `detect_marker` |
| `detectRobot` | `detect_robot` |
| `setFollowSpeed` | `set_follow_speed` |
| `setFollowDistance` | `set_follow_distance` |
| `followLine` | `follow_line` |
| `moveToMarker` | `move_to_marker` |
| `display` | `view` |

## Update pattern

Before:

```python
robot.setLEDs(255, 0, 0)
robot.cam.setVisionDebug(True)
robot.arm.moveTo(x=0, z=50)
robot.gun.moveto(yaw=90)
```

After:

```python
robot.set_leds(255, 0, 0)
robot.cam.set_vision_debug(True)
robot.arm.move_to(x=0, z=50)
robot.gun.move_to(yaw=90)
```
