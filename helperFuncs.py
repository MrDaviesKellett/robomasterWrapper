from typing import overload
from typing import Union

@overload
def clamp(val: int, minVal: int, maxVal: int) -> int: ...
@overload
def clamp(val: float, minVal: float, maxVal: float) -> float: ...


def clamp(val: Union[int , float], minVal: Union[int , float], maxVal: Union[int , float]) -> Union[int , float]:
    """
    Clamps a value between two values.
    Args:
    val (int or float): The value to be clamped.
    minVal (int or float): The minimum value.
    maxVal (int or float): The maximum value.
    Returns:
    The clamped value.
    """
    if type(val) == int and type(minVal) == int and type(maxVal) == int:
        return min(max(val, minVal), maxVal)
    elif type(val) == float and type(minVal) == float and type(maxVal) == float:
        return min(max(val, minVal), maxVal)
    else:
        raise TypeError("All arguments must be of the same type.")
