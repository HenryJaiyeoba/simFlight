"""
Utility modules for the easyflight library.
"""

from .conversions import (
    convert_units,
    normalize_angle,
    calculate_heading_error,
    meters_to_nm,
    nm_to_meters,
    calculate_distance,
    calculate_bearing
)

__all__ = [
    'convert_units',
    'normalize_angle',
    'calculate_heading_error',
    'meters_to_nm',
    'nm_to_meters',
    'calculate_distance',
    'calculate_bearing'
]
