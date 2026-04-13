# API Reference

This file documents the primary snake_case API. Deprecated aliases are listed in [migration_snake_case.md](/Volumes/StudioScratch/AI/GPTCodex/robomasterWrapper/docs/migration_snake_case.md).

## Student-first design

robowrap exists to preserve a low-friction classroom path: one robot object, intention-based method names, safe defaults, and subsystem groupings that match the physical robot. When you extend the wrapper, prefer a clear one-line path first and keep lower-level SDK details behind optional arguments or advanced subsystem facades.

## `RoboMaster`

### System and connection

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `RoboMaster(conn_type="device", protocol="tcp", serial=None, auto_reset=True, default_mode="chassis")` | `conn_type`: `device`, `station`, `usb`; `protocol`: `tcp` or `udp` | `RoboMaster` | Constructor blocks until SDK init finishes | `robot = RoboMaster()` | Raises `RuntimeError` on connection failure. Check power, link mode, then retry. |
| `close()` | none | SDK close result | n/a | `robot.close()` | Use this at the end of every script so streams and movement stop cleanly. |
| `reset()` | none | SDK reset result | SDK-defined | `robot.reset()` | If reset appears slow, wait a moment before the next command. |
| `get_module(module)` | `module`: names like `cam`, `gun`, `arm`, `battery`, `chassis` | wrapper object or raw advanced module | n/a | `camera = robot.get_module("cam")` | Raises `ValueError` for unknown module names. |
| `set_robot_mode(mode="free")` | `mode`: `free`, `chassis`, `gun` | SDK result | n/a | `robot.set_robot_mode("gun")` | Raises `ValueError` for invalid modes. |
| `get_robot_mode()` | none | `str` | n/a | `mode = robot.get_robot_mode()` | If the SDK does not report a mode, robowrap falls back to the last mode it set. |
| `get_sn()` | none | `str` | n/a | `print(robot.get_sn())` | If serial lookup fails, reconnect and retry. |
| `get_version()` | none | `str` | n/a | `print(robot.get_version())` | Use after a successful connection. |

### Audio and lights

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `play_audio(path, blocking=False, timeout=None)` | `path`: local WAV file | SDK action or completed action | Optional | `robot.play_audio("hello.wav")` | If the file is missing or invalid, fix the path or format and retry. |
| `play_sound(sound_id, times=1, blocking=False, timeout=None)` | `times`: `1..10` | SDK action or completed action | Optional | `robot.play_sound(0x101)` | Raises `ValueError` for invalid repeat counts. |
| `set_leds(r=0, g=0, b=0, leds="all", effect="on")` | RGB `0..255`; `leds`: `front`, `back`, `left`, `right`, `gun`, `gun_left`, `gun_right`, `all` | SDK result | n/a | `robot.set_leds(255, 0, 0)` | Raises `ValueError` for invalid colours or effect names. |

### Motion

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `stop()` | none | SDK result | immediate | `robot.stop()` | Use this after any continuous speed command. |
| `stop_after(duration=1.0, blocking=False)` | `duration`: `0..600 s` | `Timer` or `None` | Optional | `robot.stop_after(2)` | Raises `ValueError` for invalid durations. |
| `set_speed(x=0.0, y=0.0, z=0.0, timeout=None)` | `x,y`: `-3.5..3.5 m/s`; `z`: `-600..600 deg/s` | SDK result | continuous command, non-blocking | `robot.set_speed(x=0.5)` | Stop with `stop()` if you need to cancel continuous motion. |
| `rotate(angular_velocity=0.0, timeout=None)` | `-600..600 deg/s` | SDK result | non-blocking | `robot.rotate(45)` | Use `stop()` or `stop_after()` to end rotation. |
| `rotate_left(angular_velocity=0.0, timeout=None)` | same as `rotate` | SDK result | non-blocking | `robot.rotate_left(90)` | Same recovery as `rotate`. |
| `rotate_right(angular_velocity=0.0, timeout=None)` | same as `rotate` | SDK result | non-blocking | `robot.rotate_right(90)` | Same recovery as `rotate`. |
| `turn(angle=0.0, speed=90.0, blocking=True, timeout=None)` | `angle`: `-1800..1800 deg`; `speed`: `10..540 deg/s` | SDK action or completed action | Yes by default | `robot.turn(90)` | Raises `ValueError` on invalid angles or speeds. |
| `turn_left(angle=0.0, speed=90.0, blocking=True, timeout=None)` | same as `turn` | SDK action or completed action | Yes by default | `robot.turn_left(90)` | Same recovery as `turn`. |
| `turn_right(angle=0.0, speed=90.0, blocking=True, timeout=None)` | same as `turn` | SDK action or completed action | Yes by default | `robot.turn_right(90)` | Same recovery as `turn`. |
| `set_wheel_rpms(front_right=0, front_left=0, back_right=0, back_left=0, timeout=None)` | each wheel `-1000..1000 rpm` | SDK result | non-blocking | `robot.set_wheel_rpms(100, 100, 100, 100)` | Invalid RPMs raise `ValueError`. |
| `move(x=0.0, y=0.0, angle=0.0, speed=1.0, turn_speed=90.0, blocking=True, timeout=None)` | `x,y`: `-5..5 m`; `angle`: `-1800..1800 deg`; `speed`: `0.5..2.0 m/s`; `turn_speed`: `10..540 deg/s` | SDK action or completed action | Yes by default | `robot.move(x=0.5, y=0.2)` | Use safe units: metres and degrees. |
| `forward(distance=0.5, speed=1.0, blocking=True)` | `distance`: metres | SDK action or completed action | Yes by default | `robot.forward(0.5)` | Same validation as `move`. |
| `back(distance=0.5, speed=1.0, blocking=True)` | `distance`: metres | SDK action or completed action | Yes by default | `robot.back(0.5)` | Same validation as `move`. |
| `backward(distance=0.5, speed=1.0, blocking=True)` | alias of `back` | SDK action or completed action | Yes by default | `robot.backward(0.5)` | Same validation as `move`. |
| `left(distance=0.5, speed=1.0, blocking=True)` | `distance`: metres | SDK action or completed action | Yes by default | `robot.left(0.5)` | Same validation as `move`. |
| `right(distance=0.5, speed=1.0, blocking=True)` | `distance`: metres | SDK action or completed action | Yes by default | `robot.right(0.5)` | Same validation as `move`. |
| `circle(radius=0.25, speed=1.0, num_circles=1, blocking=True)` | `radius`: `0.05..5 m`; `speed`: `0.1..2.0 m/s` | `Timer` or `None` | Optional | `robot.circle(0.25, 1.0)` | This uses continuous speed control internally. If the robot drifts, call `stop()`. |

## `robot.chassis`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `set_pwm_value(pwm1=None ... pwm6=None)` | each PWM output `0..100 %` | SDK result | n/a | `robot.chassis.set_pwm_value(pwm1=50)` | Raises `ValueError` for invalid duty cycles. |
| `set_pwm_freq(pwm1=None ... pwm6=None)` | each PWM output `0..50000 Hz` | SDK result | n/a | `robot.chassis.set_pwm_freq(pwm1=1000)` | Raises `ValueError` for invalid frequencies. |
| `subscribe_position(callback, freq=5, coordinate_system=0)` | `freq`: `1,5,10,20,50 Hz`; coordinate system `0` or `1` | SDK subscription result | non-blocking | `robot.chassis.subscribe_position(print)` | Use `unsubscribe_position()` before re-subscribing with a different callback. |
| `unsubscribe_position()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_position()` | Use when ending a script or changing callbacks. |
| `subscribe_attitude(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_attitude(print)` | Same recovery as other subscriptions. |
| `unsubscribe_attitude()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_attitude()` |  |
| `subscribe_status(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_status(print)` |  |
| `unsubscribe_status()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_status()` |  |
| `subscribe_imu(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_imu(print)` |  |
| `unsubscribe_imu()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_imu()` |  |
| `subscribe_mode(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_mode(print)` |  |
| `unsubscribe_mode()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_mode()` |  |
| `subscribe_esc(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_esc(print)` |  |
| `unsubscribe_esc()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_esc()` |  |
| `subscribe_velocity(callback, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.chassis.subscribe_velocity(print)` |  |
| `unsubscribe_velocity()` | none | SDK result | non-blocking | `robot.chassis.unsubscribe_velocity()` |  |

## `robot.gun`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `set_leds(r=0, g=0, b=0, leds="gun", effect="on")` | RGB `0..255`; `leds`: `gun`, `gun_left`, `gun_right` | SDK result | n/a | `robot.gun.set_leds(255, 255, 0)` | Invalid LED names or colours raise `ValueError`. |
| `rotate(pitch_speed=0.0, yaw_speed=0.0)` | each speed `-360..360 deg/s` | SDK result | non-blocking | `robot.gun.rotate(yaw_speed=45)` | Use `recenter()` or another move command to recover orientation. |
| `move(pitch=0.0, yaw=0.0, pitch_speed=50.0, yaw_speed=50.0, blocking=True, timeout=None)` | `pitch,yaw`: `-55..55 deg`; speeds `0..540 deg/s` | SDK action or completed action | Yes by default | `robot.gun.move(yaw=20)` | Out-of-range values raise `ValueError`. |
| `move_to(pitch=0.0, yaw=0.0, pitch_speed=50.0, yaw_speed=50.0, blocking=True, timeout=None)` | `pitch`: `-25..30 deg`; `yaw`: `-250..250 deg` | SDK action or completed action | Yes by default | `robot.gun.move_to(yaw=90)` | Use this for startup-relative aiming. |
| `recenter(pitch_speed=100.0, yaw_speed=100.0, blocking=True, timeout=None)` | speeds `0..540 deg/s` | SDK action or completed action | Yes by default | `robot.gun.recenter()` | Good recovery step if students lose track of gimbal orientation. |
| `resume()` | none | `bool` | n/a | `robot.gun.resume()` | Use after `suspend()`. |
| `suspend()` | none | `bool` | n/a | `robot.gun.suspend()` | Leaves the gimbal unpowered. |
| `fire(fire_type="ir", times=1)` | `fire_type`: `ir` or `water`; `times`: `1..8` | SDK result | n/a | `robot.gun.fire(times=3)` | Raises `ValueError` for invalid fire types or counts. |
| `set_led(brightness=100, effect="on")` | brightness `0..255`; effect `on` or `off` | SDK result | n/a | `robot.gun.set_led(128)` | Controls the blaster LED, not the top armor LEDs. |
| `subscribe_angle(callback, freq=5)` / `unsubscribe_angle()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.gun.subscribe_angle(print)` | Unsubscribe when done. |

## `robot.arm`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `move(x=0, z=0, blocking=True, timeout=None)` | `x,z`: `-200..200 mm` | SDK action or completed action | Yes by default | `robot.arm.move(x=30, z=-20)` | Values are millimetres, not degrees. |
| `move_to(x=0, z=0, blocking=True, timeout=None)` | `x,z`: `-200..200 mm` | SDK action or completed action | Yes by default | `robot.arm.move_to(x=0, z=50)` | Use startup-relative positions. |
| `recenter(blocking=True, timeout=None)` | none | SDK action or completed action | Yes by default | `robot.arm.recenter()` | Good recovery if the arm position is unknown. |
| `reset(blocking=True, timeout=None)` | none | SDK action or completed action | Yes by default | `robot.arm.reset()` | Student-safe wrapper for recenter. |
| `subscribe_position(callback, freq=5)` / `unsubscribe_position()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.arm.subscribe_position(print)` | Unsubscribe when changing callbacks. |
| `pickup()` | uses built-in sequence | `None` | blocks through the sequence | `robot.arm.pickup()` | Requires the gripper wrapper and a reachable object. |
| `drop()` | uses built-in sequence | `None` | blocks through the sequence | `robot.arm.drop()` | Use on a clear surface. |

## `robot.gripper`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `open(power=50)` | `power`: `1..100` | SDK result | non-blocking | `robot.gripper.open()` | Reduce power if the gripper stalls. |
| `close(power=50)` | `power`: `1..100` | SDK result | non-blocking | `robot.gripper.close()` | Reduce power for delicate objects. |
| `pause()` | none | SDK result | immediate | `robot.gripper.pause()` | Safe stop for the motor. |
| `stop()` | none | SDK result | immediate | `robot.gripper.stop()` | Alias of `pause()`. |
| `reset()` | none | SDK result | immediate | `robot.gripper.reset()` | Safe wrapper that pauses the motor. |
| `subscribe_status(callback, freq=5)` / `unsubscribe_status()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.gripper.subscribe_status(print)` | Unsubscribe when done. |

## `robot.cam`

### Stream and camera utilities

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `set_resolution(resolution="360p")` | `360p`, `540p`, `720p` | `None` | n/a | `robot.cam.set_resolution("720p")` | Raises `ValueError` for unsupported resolutions. |
| `start()` / `stop()` | none | SDK result | non-blocking | `robot.cam.start()` | Stop and restart the stream if frames stall. |
| `view()` | none | current image or `None` | non-blocking display loop step | `robot.cam.view()` | Requires `opencv-python`. |
| `take_photo()` | none | SDK result | n/a | `robot.cam.take_photo()` | Ensure storage and connection are stable before shooting. |
| `start_audio_stream()` / `stop_audio_stream()` | none | SDK result | non-blocking | `robot.cam.start_audio_stream()` | Stop all media streams and retry if audio setup fails. |
| `read_audio_frame(timeout=1.0)` | `timeout`: `0.01..60 s` | audio bytes | waits up to timeout | `frame = robot.cam.read_audio_frame()` | Increase timeout if frames arrive slowly. |
| `record_audio(save_file="output.wav", seconds=5.0, sample_rate=48000)` | `seconds`: `0.1..600`; sample rate limited to common rates | SDK result | blocks for recording duration | `robot.cam.record_audio("clip.wav", 3)` | Ensure the camera audio stream is available. |
| `reset_vision()` | none | SDK result | immediate | `robot.cam.reset_vision()` | Use if switching between marker, line, and person detection modes. |

### Vision helpers

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `set_pid(p=330, i=0, d=28)` | PID gains for line following | `None` | n/a | `robot.cam.set_pid(250, 0, 20)` | Tune only after the default settings work. |
| `set_detect_mode(mode="line")` | `person`, `gesture`, `line`, `marker`, `robot` | `None` | n/a | `robot.cam.set_detect_mode("marker")` | Invalid mode names raise `ValueError`. |
| `set_vision_debug(value=True)` | boolean | `None` | n/a | `robot.cam.set_vision_debug(True)` | Use with `view()` to see overlays. |
| `set_debug_color(color=(255, 0, 0))` | RGB `0..255` | `None` | n/a | `robot.cam.set_debug_color((0, 255, 0))` | Invalid tuples raise `TypeError` or `ValueError`. |
| `stop_detect()` | none | `None` | immediate | `robot.cam.stop_detect()` | Call before switching custom callbacks manually. |
| `detect(name=None, color=None)` | defaults to current mode | SDK result | non-blocking | `robot.cam.detect("person")` | Marker and line detection need a valid colour. |
| `detect_person()` | none | SDK result | non-blocking | `robot.cam.detect_person()` | Start the video stream first for reliable results. |
| `detect_gesture()` | none | SDK result | non-blocking | `robot.cam.detect_gesture()` | Same recovery as `detect_person()`. |
| `detect_line(color="red")` | `red`, `green`, `blue` | SDK result | non-blocking | `robot.cam.detect_line("red")` | Pick the actual line colour in the classroom. |
| `detect_marker(color="red")` | `red`, `green`, `blue` | SDK result | non-blocking | `robot.cam.detect_marker("red")` | Marker colour must match the printed marker. |
| `detect_robot()` | none | SDK result | non-blocking | `robot.cam.detect_robot()` | Requires another robot in frame. |
| `set_follow_speed(speed)` | `0.05..3.5 m/s` | `None` | n/a | `robot.cam.set_follow_speed(0.4)` | Keep this slow while tuning. |
| `set_follow_distance(distance)` | `0.0..1.0` normalized target | `None` | n/a | `robot.cam.set_follow_distance(0.5)` | Use the default unless you are tuning. |
| `follow(name=None, color="red")` | currently only line following is implemented | SDK result | non-blocking | `robot.cam.follow("line")` | Raises `NotImplementedError` for unsupported targets. |
| `follow_line(color="red")` | `red`, `green`, `blue` | SDK result | non-blocking | `robot.cam.follow_line()` | Use `stop_detect()` or `robot.stop()` to interrupt. |
| `move_to_marker(marker_type="1", color="red", error=0.06, speed=1.0, min_speed=0.02, target_x=0.0, target_y=0.5)` | normalized frame positions `-1..1`; speed `0.05..3.5 m/s` | SDK subscription result | non-blocking | `robot.cam.move_to_marker("1")` | If the robot never settles, slow it down or widen `error`. |

## `robot.battery`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `subscribe(callback=None, freq=5)` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.battery.subscribe(print)` | Subscribe once before calling `get_level()`. |
| `unsubscribe()` | none | SDK result | non-blocking | `robot.battery.unsubscribe()` |  |
| `get_level()` | last subscribed value | `int` or `None` | n/a | `print(robot.battery.get_level())` | Returns `None` until a battery update has been received. |

## `robot.servo`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `move_to(index=1, angle=0, blocking=True, timeout=None)` | `index`: `1..3`; `angle`: `-180..180 deg` | SDK action or completed action | Yes by default | `robot.servo.move_to(1, 45)` | Invalid indexes or angles raise `ValueError`. |
| `set_speed(index=1, speed=0)` | `index`: `1..3`; `speed`: `-49..49` | SDK result | non-blocking | `robot.servo.set_speed(1, 10)` | Reduce speed if the servo binds. |
| `pause(index=1)` | `index`: `1..3` | SDK result | immediate | `robot.servo.pause(1)` |  |
| `get_angle(index=1)` | `index`: `1..3` | angle value | n/a | `print(robot.servo.get_angle(1))` |  |
| `subscribe(callback, freq=5)` / `unsubscribe()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.servo.subscribe(print)` |  |

## `robot.sensor`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `subscribe(callback, freq=5)` / `unsubscribe()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.sensor.subscribe(print)` | Use this for ToF distance readings. |

## `robot.sensor_adaptor`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `get_adc(board_id=1, port=1)` | `board_id`: `1..8`; `port`: `1..2` | ADC value | n/a | `robot.sensor_adaptor.get_adc(1, 1)` | Invalid IDs or ports raise `ValueError`. |
| `get_io(board_id=1, port=1)` | same ranges | IO value | n/a | `robot.sensor_adaptor.get_io(1, 1)` |  |
| `get_pulse_period(board_id=1, port=1)` | same ranges | pulse width in ms | n/a | `robot.sensor_adaptor.get_pulse_period(1, 1)` |  |
| `subscribe(callback, freq=5)` / `unsubscribe()` | `1,5,10,20,50 Hz` | SDK result | non-blocking | `robot.sensor_adaptor.subscribe(print)` |  |

## `robot.armor`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `subscribe_hit_events(callback)` / `unsubscribe_hit_events()` | none | SDK result | non-blocking | `robot.armor.subscribe_hit_events(print)` | Unsubscribe when ending a game or lesson. |
| `subscribe_ir_events(callback)` / `unsubscribe_ir_events()` | none | SDK result | non-blocking | `robot.armor.subscribe_ir_events(print)` |  |
| `set_hit_sensitivity(component="all", sensitivity=5)` | component names; sensitivity `0..10` | SDK result | n/a | `robot.armor.set_hit_sensitivity("all", 5)` | Invalid component names raise `ValueError`. |

## `robot.uart`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `configure(...)` | baud preset `0..4`; data bits `0..3`; buffers `1..4096` | SDK result | n/a | `robot.uart.configure(baud_rate=0)` | Use the default serial settings unless your extension board needs something else. |
| `send(payload)` | string, dict, tuple, or bytearray | SDK result | non-blocking | `robot.uart.send("hello")` | If a payload fails, simplify it to bytes or text first. |
| `subscribe(callback)` / `unsubscribe()` | none | SDK result | non-blocking | `robot.uart.subscribe(print)` | UART support depends on a correctly configured extension chain. |

## `robot.ai_module`

| Method | Defaults / units / ranges | Returns | Blocking | Minimal example | Failure and recovery |
| --- | --- | --- | --- | --- | --- |
| `initialize()` | none | SDK result | non-blocking | `robot.ai_module.initialize()` | Use before subscribing if you want explicit setup. |
| `subscribe(callback)` / `unsubscribe()` | none | SDK result | non-blocking | `robot.ai_module.subscribe(print)` | If no events arrive, confirm the AI module is attached and initialized. |
