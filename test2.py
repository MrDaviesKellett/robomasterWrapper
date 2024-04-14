from typing import overload
from random import randint


@overload
def hello(name: int) -> None: ...


@overload
def hello(name: str, other: str) -> None: ...


def hello(name: int | str, other: str | None = None) -> None:
    if other is not None:
        print(f"Hello {name} {other}")
    else:
        print(f"Hello {name}")


hello("joe")
hello("joe", "shmoe")
hello(123)


@overload
def setLEDs(r: int, g: int, b: int):
    """
    Set the robot LEDs to a specific colour.
    Args:
    r(int): Red value.
    g(int): Green value.
    b(int): Blue value.
    """
    ...


@overload
def setLEDs(r: int, g: int, b: int, leds: str):
    """
    Set the robot LEDs to a specific colour.
    Args:
    r(int): Red value.
    g(int): Green value.
    b(int): Blue value.
    leds (str): LED component. front, back, left, right, gimbal, gimbalLeft, gimbalRight, all. Defaults to "all".
    """
    ...


@overload
def setLEDs(r: int, g: int, b: int, effect: str):
    """
    Set the robot LEDs to a specific colour.
    Args:
    r(int): Red value.
    g(int): Green value.
    b(int): Blue value.
    effect (str): LED effect. on, off, pulse, flash, breath, scrolling. Defaults to "on".
    """
    ...


@overload
def setLEDs(r: int, g: int, b: int, *, leds: str, effect: str):
    """
    Set the robot LEDs to a specific colour.
    Args:
    r(int): Red value.
    g(int): Green value.
    b(int): Blue value.
    leds (str): LED component. front, back, left, right, gimbal, gimbalLeft, gimbalRight, all. Defaults to "all".
    effect (str): LED effect. on, off, pulse, flash, breath, scrolling. Defaults to "on".
    """
    ...


def setLEDs(
    r: int | None = None,
    g: int | None = None,
    b: int | None = None,
    leds: str | None = None,
    effect: str | None = None,
):

    if r is None or g is None or b is None:
        raise ValueError("Please specify a colour or RGB values.")

    if leds is not None:
        match leds:
            case "front":
                print("comp = led.COMP_BOTTOM_FRONT")
            case "back":
                print("comp = led.COMP_BOTTOM_BACK")
            case "left":
                print("comp = led.COMP_BOTTOM_LEFT")
            case "right":
                print("comp = led.COMP_BOTTOM_RIGHT")
            case "gimbal":
                print("comp = led.COMP_TOP_ALL")
            case "gimbalLeft":
                print("comp = led.COMP_TOP_LEFT")
            case "gimbalRight":
                print("comp = led.COMP_TOP_RIGHT")
            case "all":
                print("comp = led.COMP_ALL")
            case _:
                effect = leds

    if effect is not None:
        match effect:
            case "on":
                print("effect = led.EFFECT_ON")
            case "off":
                print("effect = led.EFFECT_OFF")
            case "pulse":
                print("effect = led.EFFECT_PULSE")
            case "flash":
                print("effect = led.EFFECT_FLASH")
            case "breath":
                print("effect = led.EFFECT_BREATH")
            case "scrolling":
                print("effect = led.EFFECT_SCROLLING")


setLEDs(255, 0, 0)
setLEDs(255, 0, 0, "front")
setLEDs(255, 0, 0, "pulse")
setLEDs(255, 0, 0, "back", "breath")
