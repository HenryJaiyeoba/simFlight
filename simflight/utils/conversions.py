"""
Utility functions for the simFlight library.
"""

import numpy as np
from typing import Dict, List, Any, Optional, Tuple, Union


def meters_to_feet(meters: float) -> float:
    """
    Convert meters to feet.
    
    Args:
        meters: Length in meters
        
    Returns:
        float: Length in feet
    """
    return meters * 3.28084


def feet_to_meters(feet: float) -> float:
    """
    Convert feet to meters.
    
    Args:
        feet: Length in feet
        
    Returns:
        float: Length in meters
    """
    return feet * 0.3048


def convert_units(value: float, from_unit: str, to_unit: str) -> float:
    """
    Convert a value from one unit to another.
    
    Args:
        value: Value to convert
        from_unit: Source unit
        to_unit: Target unit
        
    Returns:
        float: Converted value
    """
    # Define conversion factors
    conversions = {
        # Length
        'm_to_ft': 3.28084,
        'ft_to_m': 0.3048,
        
        # Speed
        'mps_to_kts': 1.94384,
        'kts_to_mps': 0.514444,
        'mps_to_fpm': 196.85,
        'fpm_to_mps': 0.00508,
        'kts_to_fpm': 101.269,
        'fpm_to_kts': 0.00987,
        
        # Angles
        'rad_to_deg': 57.2958,
        'deg_to_rad': 0.0174533,
    }
    
    # Create the conversion key
    key = f"{from_unit}_to_{to_unit}"
    
    # If direct conversion exists
    if key in conversions:
        return value * conversions[key]
    
    # If inverse conversion exists
    inverse_key = f"{to_unit}_to_{from_unit}"
    if inverse_key in conversions:
        return value / conversions[inverse_key]
    
    # If no conversion found
    raise ValueError(f"No conversion available from {from_unit} to {to_unit}")


def normalize_angle(angle: float, min_val: float = 0.0, max_val: float = 360.0) -> float:
    """
    Normalize an angle to the specified range.
    
    Args:
        angle: Angle to normalize
        min_val: Minimum value of the range
        max_val: Maximum value of the range
        
    Returns:
        float: Normalized angle
    """
    range_size = max_val - min_val
    return ((angle - min_val) % range_size) + min_val


def calculate_heading_error(current: float, target: float) -> float:
    """
    Calculate the shortest heading error between current and target headings.
    
    Args:
        current: Current heading in degrees
        target: Target heading in degrees
        
    Returns:
        float: Heading error in degrees (-180 to 180)
    """
    # Normalize angles to 0-360
    current = normalize_angle(current)
    target = normalize_angle(target)
    
    # Calculate the error
    error = target - current
    
    # Adjust for shortest path
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360
        
    return error


def meters_to_nm(meters: float) -> float:
    """
    Convert meters to nautical miles.
    
    Args:
        meters: Distance in meters
        
    Returns:
        float: Distance in nautical miles
    """
    return meters / 1852.0


def nm_to_meters(nm: float) -> float:
    """
    Convert nautical miles to meters.
    
    Args:
        nm: Distance in nautical miles
        
    Returns:
        float: Distance in meters
    """
    return nm * 1852.0


def calculate_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great circle distance between two points in nautical miles.
    
    Args:
        lat1: Latitude of point 1 in degrees
        lon1: Longitude of point 1 in degrees
        lat2: Latitude of point 2 in degrees
        lon2: Longitude of point 2 in degrees
        
    Returns:
        float: Distance in nautical miles
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat/2)**2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon/2)**2
    c = 2 * np.arcsin(np.sqrt(a))
    
    # Earth radius in nautical miles
    r = 3440.0
    
    # Calculate distance
    return c * r


def calculate_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the initial bearing from point 1 to point 2.
    
    Args:
        lat1: Latitude of point 1 in degrees
        lon1: Longitude of point 1 in degrees
        lat2: Latitude of point 2 in degrees
        lon2: Longitude of point 2 in degrees
        
    Returns:
        float: Bearing in degrees (0-360)
    """
    # Convert to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    
    # Calculate bearing
    dlon = lon2 - lon1
    y = np.sin(dlon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    bearing = np.arctan2(y, x)
    
    # Convert to degrees and normalize to 0-360
    bearing = np.degrees(bearing)
    bearing = (bearing + 360) % 360
    
    return bearing
