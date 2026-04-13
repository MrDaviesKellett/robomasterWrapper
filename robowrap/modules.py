from __future__ import annotations

from typing import Any, Callable, Optional

from .helperFuncs import ensure_choice, ensure_int_range


class _BaseModule:
    def __init__(self, robomaster: Any, module_name: str) -> None:
        self.robomaster = robomaster
        self.robot = self.robomaster.robot
        self._module_name = module_name

    @property
    def module(self) -> Any:
        return getattr(self.robot, self._module_name)


class Chassis(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "chassis")

    def set_pwm_value(
        self,
        pwm1: Optional[int] = None,
        pwm2: Optional[int] = None,
        pwm3: Optional[int] = None,
        pwm4: Optional[int] = None,
        pwm5: Optional[int] = None,
        pwm6: Optional[int] = None,
    ) -> Any:
        values = {
            "pwm1": pwm1,
            "pwm2": pwm2,
            "pwm3": pwm3,
            "pwm4": pwm4,
            "pwm5": pwm5,
            "pwm6": pwm6,
        }
        for name, value in values.items():
            if value is not None:
                ensure_int_range(name, value, 0, 100, unit="%")
        return self.module.set_pwm_value(**values)

    def set_pwm_freq(
        self,
        pwm1: Optional[int] = None,
        pwm2: Optional[int] = None,
        pwm3: Optional[int] = None,
        pwm4: Optional[int] = None,
        pwm5: Optional[int] = None,
        pwm6: Optional[int] = None,
    ) -> Any:
        values = {
            "pwm1": pwm1,
            "pwm2": pwm2,
            "pwm3": pwm3,
            "pwm4": pwm4,
            "pwm5": pwm5,
            "pwm6": pwm6,
        }
        for name, value in values.items():
            if value is not None:
                ensure_int_range(name, value, 0, 50000, unit="Hz")
        return self.module.set_pwm_freq(**values)

    def subscribe_position(self, callback: Callable[..., Any], freq: int = 5, *, coordinate_system: int = 0) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        ensure_choice("coordinate_system", coordinate_system, (0, 1))
        return self.module.sub_position(cs=coordinate_system, freq=freq, callback=callback)

    def unsubscribe_position(self) -> Any:
        return self.module.unsub_position()

    def subscribe_attitude(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_attitude(freq=freq, callback=callback)

    def unsubscribe_attitude(self) -> Any:
        return self.module.unsub_attitude()

    def subscribe_status(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_status(freq=freq, callback=callback)

    def unsubscribe_status(self) -> Any:
        return self.module.unsub_status()

    def subscribe_imu(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_imu(freq=freq, callback=callback)

    def unsubscribe_imu(self) -> Any:
        return self.module.unsub_imu()

    def subscribe_mode(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_mode(freq=freq, callback=callback)

    def unsubscribe_mode(self) -> Any:
        return self.module.unsub_mode()

    def subscribe_esc(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_esc(freq=freq, callback=callback)

    def unsubscribe_esc(self) -> Any:
        return self.module.unsub_esc()

    def subscribe_velocity(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_velocity(freq=freq, callback=callback)

    def unsubscribe_velocity(self) -> Any:
        return self.module.unsub_velocity()


class Battery(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "battery")
        self.last_level: Optional[int] = None

    def subscribe(self, callback: Optional[Callable[[int], Any]] = None, freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))

        def _callback(level: int) -> Any:
            self.last_level = level
            if callback:
                return callback(level)
            return level

        return self.module.sub_battery_info(freq=freq, callback=_callback)

    def unsubscribe(self) -> Any:
        return self.module.unsub_battery_info()

    def get_level(self) -> Optional[int]:
        return self.last_level


class Servo(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "servo")

    def move_to(self, index: int = 1, angle: int = 0, *, blocking: bool = True, timeout: Optional[float] = None) -> Any:
        ensure_int_range("index", index, 1, 3)
        ensure_int_range("angle", angle, -180, 180, unit="degrees")
        action = self.module.moveto(index=index, angle=angle)
        return self.robomaster._complete_action(action, blocking=blocking, timeout=timeout)

    def set_speed(self, index: int = 1, speed: int = 0) -> Any:
        ensure_int_range("index", index, 1, 3)
        ensure_int_range("speed", speed, -49, 49)
        return self.module.drive_speed(index=index, speed=speed)

    def pause(self, index: int = 1) -> Any:
        ensure_int_range("index", index, 1, 3)
        return self.module.pause(index=index)

    def get_angle(self, index: int = 1) -> Any:
        ensure_int_range("index", index, 1, 3)
        return self.module.get_angle(index=index)

    def subscribe(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_servo_info(freq=freq, callback=callback)

    def unsubscribe(self) -> Any:
        return self.module.unsub_servo_info()


class DistanceSensor(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "sensor")

    def subscribe(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_distance(freq=freq, callback=callback)

    def unsubscribe(self) -> Any:
        return self.module.unsub_distance()


class SensorAdaptor(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "sensor_adaptor")

    def get_adc(self, board_id: int = 1, port: int = 1) -> Any:
        ensure_int_range("board_id", board_id, 1, 8)
        ensure_int_range("port", port, 1, 2)
        return self.module.get_adc(id=board_id, port=port)

    def get_io(self, board_id: int = 1, port: int = 1) -> Any:
        ensure_int_range("board_id", board_id, 1, 8)
        ensure_int_range("port", port, 1, 2)
        return self.module.get_io(id=board_id, port=port)

    def get_pulse_period(self, board_id: int = 1, port: int = 1) -> Any:
        ensure_int_range("board_id", board_id, 1, 8)
        ensure_int_range("port", port, 1, 2)
        return self.module.get_pulse_period(id=board_id, port=port)

    def subscribe(self, callback: Callable[..., Any], freq: int = 5) -> Any:
        ensure_choice("freq", freq, (1, 5, 10, 20, 50))
        return self.module.sub_adapter(freq=freq, callback=callback)

    def unsubscribe(self) -> Any:
        return self.module.unsub_adapter()


class Armor(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "armor")

    def subscribe_hit_events(self, callback: Callable[..., Any]) -> Any:
        return self.module.sub_hit_event(callback=callback)

    def unsubscribe_hit_events(self) -> Any:
        return self.module.unsub_hit_event()

    def subscribe_ir_events(self, callback: Callable[..., Any]) -> Any:
        return self.module.sub_ir_event(callback=callback)

    def unsubscribe_ir_events(self) -> Any:
        return self.module.unsub_ir_event()

    def set_hit_sensitivity(self, component: str = "all", sensitivity: int = 5) -> Any:
        ensure_choice(
            "component",
            component,
            (
                "all",
                "top_all",
                "bottom_all",
                "top_left",
                "top_right",
                "bottom_left",
                "bottom_right",
                "bottom_front",
                "bottom_back",
            ),
        )
        ensure_int_range("sensitivity", sensitivity, 0, 10)
        return self.module.set_hit_sensitivity(comp=component, sensitivity=sensitivity)


class Uart(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "uart")

    def configure(
        self,
        *,
        baud_rate: int = 0,
        data_bit: int = 1,
        odd_even: int = 0,
        stop_bit: int = 0,
        rx_enabled: int = 1,
        tx_enabled: int = 1,
        rx_size: int = 50,
        tx_size: int = 50,
    ) -> Any:
        ensure_int_range("baud_rate", baud_rate, 0, 4)
        ensure_int_range("data_bit", data_bit, 0, 3)
        ensure_int_range("odd_even", odd_even, 0, 3)
        ensure_int_range("stop_bit", stop_bit, 0, 2)
        ensure_int_range("rx_enabled", rx_enabled, 0, 1)
        ensure_int_range("tx_enabled", tx_enabled, 0, 1)
        ensure_int_range("rx_size", rx_size, 1, 4096)
        ensure_int_range("tx_size", tx_size, 1, 4096)
        return self.module.serial_param_set(
            baud_rate=baud_rate,
            data_bit=data_bit,
            odd_even=odd_even,
            stop_bit=stop_bit,
            rx_en=rx_enabled,
            tx_en=tx_enabled,
            rx_size=rx_size,
            tx_size=tx_size,
        )

    def send(self, payload: Any) -> Any:
        return self.module.serial_send_msg(payload)

    def subscribe(self, callback: Callable[..., Any]) -> Any:
        return self.module.sub_serial_msg(callback, (), {})

    def unsubscribe(self) -> Any:
        return self.module.unsub_serial_msg()


class AiModule(_BaseModule):
    def __init__(self, robomaster: Any) -> None:
        super().__init__(robomaster, "ai_module")

    def initialize(self) -> Any:
        return self.module.init_ai_module()

    def subscribe(self, callback: Callable[..., Any]) -> Any:
        return self.module.sub_ai_event(callback=callback)

    def unsubscribe(self) -> Any:
        return self.module.unsub_ai_event()
