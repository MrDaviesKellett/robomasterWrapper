# Troubleshooting

## Connection problems

### `Robot could not be connected`

- Check that the EP is powered on.
- Confirm the connection mode matches your code: `device` maps to AP mode and `usb` maps to RNDIS.
- If you are using a specific serial number, confirm it matches the robot.
- Recovery: restart the robot, reconnect Wi-Fi or USB, and run the script again.

### The script hangs during startup

- The RoboMaster SDK can take a few seconds to initialize.
- Recovery: disconnect and reconnect the control link, then rerun with only one controller script active.

### `No module named 'robomaster'`

- robowrap expects the RoboMaster SDK Python module to be installed.
- Recovery: install the maintained package with `python -m pip install robomaster-sdk-modern`.

## Camera problems

### OpenCV and PyAV duplicate FFmpeg warnings

- The modern SDK uses PyAV for media. OpenCV also ships FFmpeg libraries.
- On macOS, importing both can produce duplicate `AVFFrameReceiver` or `AVFAudioReceiver` warnings and may cause corrupt frames or crashes.
- Recovery: use `robot.cam.view()` for display. robowrap uses PySide6 for the viewer and does not use OpenCV.
- Use `frame = robot.cam.frame()` when your code needs the latest frame without a display window.

### Video stream starts but nothing appears

- Make sure you called `robot.cam.view()` or `robot.cam.start()` before reading frames.
- On some classroom machines another process may still hold the stream.
- Recovery: call `robot.cam.stop()`, wait one second, then restart the stream.

### Audio stream fails

- Audio capture depends on the EP media stream support in the installed SDK.
- Recovery: stop video and audio streams, then start only the audio stream once.

## Vision problems

### Detection does not return any boxes

- Vision needs an active camera stream.
- Marker and line detection also need the correct color.
- Recovery: call `robot.cam.start()`, then `detect_line("red")` or `detect_marker("red")` explicitly.

### `move_to_marker` never reaches the target

- The marker may be too far away, too close, or partially occluded.
- Recovery: slow the robot down, increase the `error` margin slightly, or move the robot closer before calling `move_to_marker`.

### `follow` raises `NotImplementedError`

- The student wrapper only implements line following.
- Recovery: use `follow_line()` instead of `follow("marker")` or `follow("person")`.

## Action problems

### An arm or gimbal movement returns too early

- Position-based helpers block by default, but speed-based helpers do not.
- Recovery: use movement helpers like `move_to`, `forward`, or `turn_left` for blocking behavior, or keep the returned action object and wait for it yourself.

### The robot keeps moving after a speed command

- `set_speed` is continuous speed control.
- Recovery: call `stop()` or use `stop_after(duration)` if you want timed motion.

## Invalid parameter errors

- robowrap now raises `ValueError` for out-of-range values instead of silently clamping them.
- Recovery: check the units in [api_reference.md](api_reference.md) and pass values inside the documented range.
