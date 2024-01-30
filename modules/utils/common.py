from typing import Any


def get_class_name(cls) -> str:
    """Return class name"""
    return f"{cls.__name__}"

def any_to_str(val: Any) -> str:
    """Return the class name or the class name of the object, or 'val' if it's a string type."""
    if isinstance(val, str):
        return val
    elif not callable(val):
        return get_class_name(type(val))
    else:
        return get_class_name(val)