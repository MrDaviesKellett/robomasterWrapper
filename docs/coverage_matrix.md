# Coverage Matrix

Derived from the official DJI RoboMaster EP SDK source on `2026-04-13` and limited to the EP `Robot` surface, not the Tello `Drone` surface.

Status values:

- `wrapped`: exposed through the student wrapper
- `intentionally_omitted`: deliberately not promoted to the public wrapper API

## Robot

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `Robot.chassis` | `robot.chassis` | wrapped | Advanced chassis subscriptions and PWM controls live here. |
| `Robot.gimbal` | `robot.gun` | wrapped | Student-first gimbal wrapper. |
| `Robot.blaster` | `robot.gun` | wrapped | Blaster controls live on `Gun`. |
| `Robot.led` | `robot.set_leds()` / `robot.gun.set_leds()` | wrapped | Top and body lights exposed with plain names. |
| `Robot.vision` | `robot.cam` | wrapped | Vision is intentionally grouped under camera. |
| `Robot.battery` | `robot.battery` | wrapped | Battery facade caches last level. |
| `Robot.camera` | `robot.cam` | wrapped | Video, photo, and audio functions live here. |
| `Robot.robotic_arm` | `robot.arm` | wrapped | Arm motion and subscriptions. |
| `Robot.gripper` | `robot.gripper` | wrapped | Gripper motion and status subscriptions. |
| `Robot.servo` | `robot.servo` | wrapped | Servo facade. |
| `Robot.sensor` | `robot.sensor` | wrapped | Distance sensor facade. |
| `Robot.sensor_adaptor` | `robot.sensor_adaptor` | wrapped | Sensor adaptor facade. |
| `Robot.armor` | `robot.armor` | wrapped | Armor event facade. |
| `Robot.uart` | `robot.uart` | wrapped | UART facade. |
| `Robot.ai_module` | `robot.ai_module` | wrapped | AI module facade. |
| `Robot.dds` | raw only via `get_module("dds")` | intentionally_omitted | DDS is too low-level for the public student API. |
| `Robot.initialize(...)` | `RoboMaster()` | wrapped | Connection setup happens in the constructor. |
| `Robot.close()` | `robot.close()` | wrapped | Clean shutdown. |
| `Robot.reset()` | `robot.reset()` | wrapped | Safe reset path. |
| `Robot.reset_robot_mode()` | none | intentionally_omitted | Internal recovery detail. |
| `Robot.set_robot_mode(...)` | `robot.set_robot_mode(...)` | wrapped | Uses `free`, `chassis`, `gun`. |
| `Robot.get_robot_mode()` | `robot.get_robot_mode()` | wrapped | Returns student-facing mode names. |
| `Robot.get_version()` | `robot.get_version()` | wrapped | Firmware version. |
| `Robot.get_sn()` | `robot.get_sn()` | wrapped | Hardware serial. |
| `Robot.play_audio(...)` | `robot.play_audio(...)` | wrapped | Local audio upload + playback. |
| `Robot.play_sound(...)` | `robot.play_sound(...)` | wrapped | System sound playback. |

## Chassis

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `drive_speed` | `robot.set_speed(...)` | wrapped | Continuous speed control. |
| `drive_wheels` | `robot.set_wheel_rpms(...)` | wrapped | Direct wheel control. |
| `move` | `robot.move(...)`, `forward`, `back`, `left`, `right`, `turn` | wrapped | Position helpers on root object. |
| `set_pwm_value` | `robot.chassis.set_pwm_value(...)` | wrapped | Advanced expansion control. |
| `set_pwm_freq` | `robot.chassis.set_pwm_freq(...)` | wrapped | Advanced expansion control. |
| `sub_position` / `unsub_position` | `robot.chassis.subscribe_position()` / `unsubscribe_position()` | wrapped | |
| `sub_attitude` / `unsub_attitude` | `robot.chassis.subscribe_attitude()` / `unsubscribe_attitude()` | wrapped | |
| `sub_status` / `unsub_status` | `robot.chassis.subscribe_status()` / `unsubscribe_status()` | wrapped | |
| `sub_imu` / `unsub_imu` | `robot.chassis.subscribe_imu()` / `unsubscribe_imu()` | wrapped | |
| `sub_mode` / `unsub_mode` | `robot.chassis.subscribe_mode()` / `unsubscribe_mode()` | wrapped | |
| `sub_esc` / `unsub_esc` | `robot.chassis.subscribe_esc()` / `unsubscribe_esc()` | wrapped | |
| `sub_velocity` / `unsub_velocity` | `robot.chassis.subscribe_velocity()` / `unsubscribe_velocity()` | wrapped | |
| `stick_overlay` | none | intentionally_omitted | Advanced RC blending is outside the student-first surface. |
| `_sub_sbus` / `_unsub_sbus` | none | intentionally_omitted | Private RC bus API. |

## Gimbal and blaster

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `gimbal.drive_speed` | `robot.gun.rotate(...)` | wrapped | |
| `gimbal.move` | `robot.gun.move(...)` | wrapped | Relative movement. |
| `gimbal.moveto` | `robot.gun.move_to(...)` | wrapped | Absolute movement. |
| `gimbal.recenter` | `robot.gun.recenter()` | wrapped | |
| `gimbal.resume` / `suspend` | `robot.gun.resume()` / `suspend()` | wrapped | |
| `gimbal.sub_angle` / `unsub_angle` | `robot.gun.subscribe_angle()` / `unsubscribe_angle()` | wrapped | |
| `gimbal._set_work_mode` | none | intentionally_omitted | Internal work-mode detail. |
| `blaster.fire` | `robot.gun.fire(...)` | wrapped | |
| `blaster.set_led` | `robot.gun.set_led(...)` | wrapped | |

## Camera and vision

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `camera.start_video_stream` / `stop_video_stream` | `robot.cam.start()` / `stop()` | wrapped | |
| `camera.read_cv2_image` | `robot.cam.view()` / `robot.cam.frame()` | wrapped | `view()` opens a PySide6 live viewer; `frame()` returns the latest frame. |
| `camera.take_photo` | `robot.cam.take_photo()` | wrapped | |
| `camera.start_audio_stream` / `stop_audio_stream` | `robot.cam.start_audio_stream()` / `stop_audio_stream()` | wrapped | |
| `camera.read_audio_frame` | `robot.cam.read_audio_frame()` | wrapped | |
| `camera.record_audio` | `robot.cam.record_audio()` | wrapped | |
| `camera._set_zoom` | none | intentionally_omitted | Low-level camera tuning omitted. |
| `vision.reset` | `robot.cam.reset_vision()` | wrapped | |
| `vision.sub_detect_info` / `unsub_detect_info` | `detect*`, `follow_line`, `move_to_marker`, `stop_detect` | wrapped | Friendly helpers keep vision grouped under camera. |
| `vision._enable_detection` / `_disable_detection` / `_get_sdk_function` / `_set_color` | none | intentionally_omitted | Internal vision control details are hidden. |

## Arm and gripper

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `robotic_arm.move` | `robot.arm.move(...)` | wrapped | |
| `robotic_arm.moveto` | `robot.arm.move_to(...)` | wrapped | |
| `robotic_arm.recenter` | `robot.arm.recenter()` / `reset()` | wrapped | |
| `robotic_arm.sub_position` / `unsub_position` | `robot.arm.subscribe_position()` / `unsubscribe_position()` | wrapped | |
| `robotic_arm.reset` | `robot.arm.reset()` | wrapped | Student wrapper maps reset to recenter because upstream reset is a no-op. |
| `gripper.open` / `close` / `pause` | `robot.gripper.open()` / `close()` / `pause()` | wrapped | |
| `gripper.sub_status` / `unsub_status` | `robot.gripper.subscribe_status()` / `unsubscribe_status()` | wrapped | |
| `gripper.reset` | `robot.gripper.reset()` | wrapped | Student wrapper maps reset to a safe pause. |

## Battery, servo, sensors, armor, UART, AI

| Upstream EP API | Wrapper surface | Status | Notes |
| --- | --- | --- | --- |
| `battery.sub_battery_info` / `unsub_battery_info` | `robot.battery.subscribe()` / `unsubscribe()` | wrapped | |
| `servo.moveto` | `robot.servo.move_to(...)` | wrapped | |
| `servo.drive_speed` | `robot.servo.set_speed(...)` | wrapped | |
| `servo.pause` | `robot.servo.pause(...)` | wrapped | |
| `servo.get_angle` | `robot.servo.get_angle(...)` | wrapped | |
| `servo.sub_servo_info` / `unsub_servo_info` | `robot.servo.subscribe()` / `unsubscribe()` | wrapped | |
| `sensor.sub_distance` / `unsub_distance` | `robot.sensor.subscribe()` / `unsubscribe()` | wrapped | |
| `sensor_adaptor.get_adc` | `robot.sensor_adaptor.get_adc(...)` | wrapped | |
| `sensor_adaptor.get_io` | `robot.sensor_adaptor.get_io(...)` | wrapped | |
| `sensor_adaptor.get_pulse_period` | `robot.sensor_adaptor.get_pulse_period(...)` | wrapped | |
| `sensor_adaptor.sub_adapter` / `unsub_adapter` | `robot.sensor_adaptor.subscribe()` / `unsubscribe()` | wrapped | |
| `armor.sub_hit_event` / `unsub_hit_event` | `robot.armor.subscribe_hit_events()` / `unsubscribe_hit_events()` | wrapped | |
| `armor.sub_ir_event` / `unsub_ir_event` | `robot.armor.subscribe_ir_events()` / `unsubscribe_ir_events()` | wrapped | |
| `armor.set_hit_sensitivity` | `robot.armor.set_hit_sensitivity(...)` | wrapped | |
| `uart.serial_param_set` | `robot.uart.configure(...)` | wrapped | |
| `uart.serial_send_msg` | `robot.uart.send(...)` | wrapped | |
| `uart.sub_serial_msg` / `unsub_serial_msg` | `robot.uart.subscribe()` / `unsubscribe()` | wrapped | |
| `uart.serial_read_data` | none | intentionally_omitted | Upstream implementation is incomplete. |
| `ai_module.init_ai_module` | `robot.ai_module.initialize()` | wrapped | |
| `ai_module.sub_ai_event` / `unsub_ai_event` | `robot.ai_module.subscribe()` / `unsubscribe()` | wrapped | |
