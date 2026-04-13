from __future__ import annotations

import functools
import re
import warnings
from typing import Any, Callable, Iterable, Optional, TypeVar, Union

Number = Union[int, float]
T = TypeVar("T")

SNAKE_CASE_RE = re.compile(r"^[a-z_][a-z0-9_]*$")


def clamp(val: Number, min_val: Number, max_val: Number) -> Number:
    """Clamp a numeric value to an inclusive range."""
    if min_val > max_val:
        raise ValueError("min_val must be less than or equal to max_val.")
    if not isinstance(val, (int, float)):
        raise TypeError("val must be an int or float.")
    if not isinstance(min_val, (int, float)) or not isinstance(max_val, (int, float)):
        raise TypeError("min_val and max_val must be ints or floats.")
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
        raise TypeError(f"{name} must be a number.")
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
        raise TypeError(f"{name} must be an int.")
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
        raise TypeError(f"{name} must be an (r, g, b) tuple.")
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
    if timeout is None:
        return waiter()
    return waiter(timeout)


def deprecated_alias(new_name: str) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
    """Build a deprecated alias that forwards to a snake_case method."""

    def decorator(func: Callable[..., Any]) -> Callable[..., Any]:
        old_name = func.__name__

        @functools.wraps(func)
        def wrapper(self: Any, *args: Any, **kwargs: Any) -> Any:
            warnings.warn(
                (
                    f"{old_name} is deprecated and will be removed in robowrap 0.9.0 "
                    f"on 2026-09-01. Use {new_name} instead."
                ),
                DeprecationWarning,
                stacklevel=2,
            )
            return getattr(self, new_name)(*args, **kwargs)

        return wrapper

    return decorator
