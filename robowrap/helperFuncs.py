from __future__ import annotations

import re
from typing import Any, Callable, Iterable, Optional, TypeVar, Union

Number = Union[int, float]
T = TypeVar("T")

SNAKE_CASE_RE = re.compile(r"^[a-z_][a-z0-9_]*$")


def clamp(val: Number, min_val: Number, max_val: Number) -> Number:
    """Clamp a numeric value to an inclusive range."""
    for name, value in (("val", val), ("min_val", min_val), ("max_val", max_val)):
        if not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be of type int or float.")
    if min_val > max_val:
        raise ValueError("min_val must be less than or equal to max_val.")
    return min(max(val, min_val), max_val)


def ensure_range(
    name: str,
    value: Number,
    minimum: Number,
    maximum: Number,
    *,
    unit: str = "",
) -> Number:
    """Validate that a numeric value falls inside an inclusive range."""
    if not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be of type int or float.")
    if value < minimum or value > maximum:
        suffix = f" {unit}".rstrip()
        raise ValueError(f"{name} must be between {minimum} and {maximum}{suffix}.")
    return value


def ensure_int_range(
    name: str,
    value: int,
    minimum: int,
    maximum: int,
    *,
    unit: str = "",
) -> int:
    """Validate that an integer falls inside an inclusive range."""
    if not isinstance(value, int):
        raise TypeError(f"{name} must be of type int.")
    ensure_range(name, value, minimum, maximum, unit=unit)
    return value


def ensure_choice(name: str, value: T, choices: Iterable[T]) -> T:
    """Validate that a value is one of a finite set of choices."""
    options = tuple(choices)
    if value not in options:
        joined = ", ".join(str(option) for option in options)
        raise ValueError(f"{name} must be one of: {joined}.")
    return value


def ensure_color(name: str, value: tuple[int, int, int]) -> tuple[int, int, int]:
    """Validate an RGB tuple."""
    if not isinstance(value, tuple) or len(value) != 3:
        raise TypeError(f"{name} must be of type tuple with three int values (r, g, b).")
    red, green, blue = value
    ensure_int_range(f"{name}[0]", red, 0, 255)
    ensure_int_range(f"{name}[1]", green, 0, 255)
    ensure_int_range(f"{name}[2]", blue, 0, 255)
    return red, green, blue


def wait_for_action(action: Any, blocking: bool = True, timeout: Optional[float] = None) -> Any:
    """Return an action immediately or wait for completion when supported."""
    if not blocking:
        return action
    waiter = getattr(action, "wait_for_completed", None)
    if not callable(waiter):
        return action
    completed = waiter() if timeout is None else waiter(timeout)
    return bool(completed) and not getattr(action, "has_failed", False)

