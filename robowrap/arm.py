from __future__ import annotations

from typing import Any, Callable, Optional

from .helperFuncs import deprecated_alias, ensure_choice, ensure_int_range


class Arm:
    def __init__(self, robomaster: Any) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot
        self.gripper = getattr(self.robomaster, "gripper", None)

    def move(
        self,
        x: int = 0,
        z: int = 0,
        *,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        """Move the arm relative to its current position in millimetres."""
        ensure_int_range("x", x, -200, 200, unit="mm")
        ensure_int_range("z", z, -200, 200, unit="mm")
        action = self.robot.robotic_arm.move(x=x, y=z)
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def move_to(
        self,
        x: int = 0,
        z: int = 0,
        *,
        blocking: bool = True,
        timeout: Optional[float] = None,
    ) -> Any:
        """Move the arm to an absolute position measured from startup in millimetres."""
        ensure_int_range("x", x, -200, 200, unit="mm")
        ensure_int_range("z", z, -200, 200, unit="mm")
        action = self.robot.robotic_arm.moveto(x=x, y=z)
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def recenter(self, *, blocking: bool = True, timeout: Optional[float] = None) -> Any:
        action = self.robot.robotic_arm.recenter()
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def reset(self, *, blocking: bool = True, timeout: Optional[float] = None) -> Any:
        """Return the arm to its startup position."""
        return self.recenter(blocking=blocking, timeout=timeout)

    def subscribe_position(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.robot.robotic_arm.sub_position(freq=freq, callback=callback)

    def unsubscribe_position(self) -> Any:
        return self.robot.robotic_arm.unsub_position()

    def pickup(self) -> None:
        """Run a simple classroom pickup sequence."""
        if self.gripper is None:
            raise RuntimeError("The gripper wrapper is not available.")
        self.move_to(x=0, z=50)
        self.gripper.open()
        self.robomaster.sleep(1.5)
        self.move(x=175, z=-200)
        self.gripper.close()
        self.robomaster.sleep(1.5)
        self.gripper.pause()
        self.move_to(x=0, z=50)

    def drop(self) -> None:
        """Run a simple classroom drop sequence."""
        if self.gripper is None:
            raise RuntimeError("The gripper wrapper is not available.")
        self.move(x=175, z=-200)
        self.gripper.open()
        self.robomaster.sleep(1.5)
        self.move_to(x=0, z=50)
        self.gripper.pause()

    @deprecated_alias("move_to")
    def moveTo(self, *args: Any, **kwargs: Any) -> Any:
        return None


class Gripper:
    def __init__(self, robomaster: Any) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot

    def open(self, power: int = 50) -> Any:
        ensure_int_range("power", power, 1, 100)
        return self.robot.gripper.open(power=power)

    def close(self, power: int = 50) -> Any:
        ensure_int_range("power", power, 1, 100)
        return self.robot.gripper.close(power=power)

    def pause(self) -> Any:
        return self.robot.gripper.pause()

    def stop(self) -> Any:
        return self.pause()

    def reset(self) -> Any:
        """Stop the gripper motor and leave the claw in a safe passive state."""
        return self.pause()

    def subscribe_status(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.robot.gripper.sub_status(freq=freq, callback=callback)

    def unsubscribe_status(self) -> Any:
        return self.robot.gripper.unsub_status()
