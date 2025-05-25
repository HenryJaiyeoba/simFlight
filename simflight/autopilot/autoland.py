"""
Autoland module for aircraft control systems.

This module provides an automatic landing system that integrates with
existing autopilot subsystems to perform approach and landing operations.
"""

from typing import Dict, List, Tuple, Optional, Union
import numpy as np
from math import radians, cos, sin, asin, sqrt, atan2, degrees

from ..autopilot.base import AutopilotSubsystem
from ..controllers.pid_controller import PIDController
from ..autopilot.heading_hold import HeadingHold
from ..autopilot.altitude_hold import AltitudeHold
from ..autopilot.speed_hold import SpeedHold


class GlideSlope(AutopilotSubsystem):
    """
    Glide slope control for approach and landing.
    
    This subsystem manages the vertical path during approach by following
    a specified glide slope angle.
    
    Attributes:
        name (str): Name of the autopilot subsystem
        controller (PIDController): PID controller for glide slope
        enabled (bool): Whether the subsystem is currently enabled
        target (float): Target glide slope angle in degrees
        runway_altitude (float): Runway elevation in feet
        runway_heading (float): Runway heading in degrees
        runway_threshold_lat (float): Latitude of runway threshold
        runway_threshold_lon (float): Longitude of runway threshold
    """
    
    def __init__(
        self,
        kp: float = 0.02,
        ki: float = 0.002,
        kd: float = 0.1,
        target_glide_slope: float = 3.0,  # degrees
        output_limits: Optional[Tuple[float, float]] = (-0.7, 0.3)
    ):
        """
        Initialize the glide slope controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            target_glide_slope: Target glide slope angle in degrees
            output_limits: Min and max elevator control values
        """
        super().__init__(
            name="Glide Slope",
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=output_limits
        )
        
        self.target = target_glide_slope
        self.runway_altitude = 0.0
        self.runway_heading = 0.0
        self.runway_threshold_lat = 0.0
        self.runway_threshold_lon = 0.0
        self.distance_to_threshold = float('inf')
        self.current_glide_slope = 0.0
    
    def set_runway(
        self,
        threshold_lat: float,
        threshold_lon: float,
        altitude: float,
        heading: float
    ) -> None:
        """
        Set the runway parameters for approach.
        
        Args:
            threshold_lat: Latitude of runway threshold in degrees
            threshold_lon: Longitude of runway threshold in degrees
            altitude: Runway elevation in feet
            heading: Runway heading in degrees
        """
        self.runway_threshold_lat = threshold_lat
        self.runway_threshold_lon = threshold_lon
        self.runway_altitude = altitude
        self.runway_heading = heading
    
    def _haversine_distance(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float
    ) -> float:
        """
        Calculate the great circle distance between two points.
        
        Args:
            lat1: Latitude of point 1 in degrees
            lon1: Longitude of point 1 in degrees
            lat2: Latitude of point 2 in degrees
            lon2: Longitude of point 2 in degrees
            
        Returns:
            Distance between the points in meters
        """
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        r = 6371000  # Radius of earth in meters
        
        return c * r
    
    def _calculate_current_glide_slope(
        self,
        current_altitude: float,
        runway_altitude: float,
        distance_to_threshold: float
    ) -> float:
        """
        Calculate the current glide slope angle.
        
        Args:
            current_altitude: Current aircraft altitude in feet
            runway_altitude: Runway elevation in feet
            distance_to_threshold: Distance to runway threshold in meters
            
        Returns:
            Current glide slope angle in degrees
        """
        # Convert distance to feet for consistent units
        distance_feet = distance_to_threshold * 3.28084
        
        # Calculate altitude difference
        altitude_diff = current_altitude - runway_altitude
        
        # Avoid division by zero
        if distance_feet < 1.0:
            return 0.0
        
        # Calculate glide slope angle
        glide_slope = degrees(np.arctan2(altitude_diff, distance_feet))
        
        return glide_slope
    
    def update(self, aircraft) -> float:
        """
        Update the glide slope controller and return control signal.
        
        Args:
            aircraft: The aircraft being controlled
            
        Returns:
            Elevator control signal
        """
        if not self.enabled:
            return 0.0
        
        # Calculate distance to runway threshold
        current_lat = aircraft.state['latitude']
        current_lon = aircraft.state['longitude']
        
        self.distance_to_threshold = self._haversine_distance(
            current_lat, current_lon,
            self.runway_threshold_lat, self.runway_threshold_lon
        )
        
        # Calculate current glide slope
        current_altitude = aircraft.state['altitude']
        self.current_glide_slope = self._calculate_current_glide_slope(
            current_altitude,
            self.runway_altitude,
            self.distance_to_threshold
        )
        
        # Calculate error (desired glide slope - current glide slope)
        # Note: positive glide slope means descending
        error = self.target - self.current_glide_slope
        
        # Compute PID control signal
        elevator = self.controller.compute(error)
        
        return elevator
    
    def get_error(self) -> float:
        """
        Get the current glide slope error.
        
        Returns:
            The difference between target and current glide slope in degrees
        """
        return self.target - self.current_glide_slope
    
    def get_status(self) -> Dict:
        """
        Get the current status of the glide slope controller.
        
        Returns:
            A dictionary with the current glide slope status
        """
        return {
            'enabled': self.enabled,
            'target_glide_slope': self.target,
            'current_glide_slope': self.current_glide_slope,
            'distance_to_threshold': self.distance_to_threshold,
            'error': self.get_error(),
            'runway_altitude': self.runway_altitude,
            'runway_heading': self.runway_heading,
            'runway_threshold_lat': self.runway_threshold_lat,
            'runway_threshold_lon': self.runway_threshold_lon
        }
    
    def compute(self, aircraft_state: Dict[str, float]) -> Dict[str, float]:
        """
        Compute control commands based on current aircraft state.
        
        Args:
            aircraft_state: Current aircraft state
            
        Returns:
            dict: Control commands to apply to the aircraft
        """
        if not self.enabled:
            return {'elevator': 0.0}
        
        # Calculate distance to runway threshold
        current_lat = aircraft_state['latitude']
        current_lon = aircraft_state['longitude']
        
        self.distance_to_threshold = self._haversine_distance(
            current_lat, current_lon,
            self.runway_threshold_lat, self.runway_threshold_lon
        )
        
        # Calculate current glide slope
        current_altitude = aircraft_state['altitude']
        self.current_glide_slope = self._calculate_current_glide_slope(
            current_altitude,
            self.runway_altitude,
            self.distance_to_threshold
        )
        
        # Calculate error (desired glide slope - current glide slope)
        # Note: positive glide slope means descending
        error = self.target - self.current_glide_slope
        
        # Compute PID control signal
        elevator = self.controller.compute(error)
        
        return {'elevator': elevator}


class Localizer(AutopilotSubsystem):
    """
    Localizer control for approach and landing.
    
    This subsystem manages the lateral path during approach by following
    the runway centerline.
    
    Attributes:
        name (str): Name of the autopilot subsystem
        controller (PIDController): PID controller for localizer
        enabled (bool): Whether the subsystem is currently enabled
        target (float): Target localizer deviation (should be 0)
        runway_heading (float): Runway heading in degrees
        runway_threshold_lat (float): Latitude of runway threshold
        runway_threshold_lon (float): Longitude of runway threshold
    """
    
    def __init__(
        self,
        kp: float = 0.02,
        ki: float = 0.001,
        kd: float = 0.1,
        output_limits: Optional[Tuple[float, float]] = (-0.5, 0.5)
    ):
        """
        Initialize the localizer controller.
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Min and max aileron control values
        """
        super().__init__(
            name="Localizer",
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=output_limits
        )
        
        self.target = 0.0  # Target is always zero deviation
        self.runway_heading = 0.0
        self.runway_threshold_lat = 0.0
        self.runway_threshold_lon = 0.0
        self.lateral_deviation = 0.0
    
    def set_runway(
        self,
        threshold_lat: float,
        threshold_lon: float,
        heading: float
    ) -> None:
        """
        Set the runway parameters for approach.
        
        Args:
            threshold_lat: Latitude of runway threshold in degrees
            threshold_lon: Longitude of runway threshold in degrees
            heading: Runway heading in degrees
        """
        self.runway_threshold_lat = threshold_lat
        self.runway_threshold_lon = threshold_lon
        self.runway_heading = heading
    
    def _calculate_bearing(
        self,
        lat1: float,
        lon1: float,
        lat2: float,
        lon2: float
    ) -> float:
        """
        Calculate the initial bearing between two points.
        
        Args:
            lat1: Latitude of point 1 in degrees
            lon1: Longitude of point 1 in degrees
            lat2: Latitude of point 2 in degrees
            lon2: Longitude of point 2 in degrees
            
        Returns:
            Initial bearing in degrees (0-360)
        """
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        # Calculate bearing
        dlon = lon2 - lon1
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        initial_bearing = atan2(y, x)
        
        # Convert to degrees and normalize
        initial_bearing = degrees(initial_bearing)
        compass_bearing = (initial_bearing + 360) % 360
        
        return compass_bearing
    
    def _calculate_lateral_deviation(
        self,
        current_lat: float,
        current_lon: float,
        runway_lat: float,
        runway_lon: float,
        runway_heading: float
    ) -> float:
        """
        Calculate the lateral deviation from the runway centerline.
        
        Args:
            current_lat: Current aircraft latitude in degrees
            current_lon: Current aircraft longitude in degrees
            runway_lat: Runway threshold latitude in degrees
            runway_lon: Runway threshold longitude in degrees
            runway_heading: Runway heading in degrees
            
        Returns:
            Lateral deviation in meters (positive is right of centerline)
        """
        # Calculate bearing from threshold to aircraft
        bearing_to_aircraft = self._calculate_bearing(
            runway_lat, runway_lon,
            current_lat, current_lon
        )
        
        # Calculate bearing difference (normalized to ±180°)
        bearing_diff = ((bearing_to_aircraft - runway_heading + 180) % 360) - 180
        
        # Calculate distance between points (in meters)
        lat1, lon1, lat2, lon2 = map(radians, [runway_lat, runway_lon, current_lat, current_lon])
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a))
        distance = c * 6371000  # Earth radius in meters
        
        # Calculate lateral deviation
        lateral_dev = distance * sin(radians(bearing_diff))
        
        return lateral_dev
    
    def update(self, aircraft) -> float:
        """
        Update the localizer controller and return control signal.
        
        Args:
            aircraft: The aircraft being controlled
            
        Returns:
            Aileron control signal
        """
        if not self.enabled:
            return 0.0
        
        # Calculate lateral deviation from runway centerline
        current_lat = aircraft.state['latitude']
        current_lon = aircraft.state['longitude']
        
        self.lateral_deviation = self._calculate_lateral_deviation(
            current_lat, current_lon,
            self.runway_threshold_lat, self.runway_threshold_lon,
            self.runway_heading
        )
        
        # Error is the lateral deviation (target is 0)
        error = self.lateral_deviation
        
        # Compute PID control signal
        aileron = self.controller.compute(error)
        
        return aileron
    
    def get_error(self) -> float:
        """
        Get the current localizer error.
        
        Returns:
            The lateral deviation from the runway centerline in meters
        """
        return self.lateral_deviation
    
    def get_status(self) -> Dict:
        """
        Get the current status of the localizer controller.
        
        Returns:
            A dictionary with the current localizer status
        """
        return {
            'enabled': self.enabled,
            'lateral_deviation': self.lateral_deviation,
            'error': self.get_error(),
            'runway_heading': self.runway_heading,
            'runway_threshold_lat': self.runway_threshold_lat,
            'runway_threshold_lon': self.runway_threshold_lon
        }
    
    def compute(self, aircraft_state: Dict[str, float]) -> Dict[str, float]:
        """
        Compute control commands based on current aircraft state.
        
        Args:
            aircraft_state: Current aircraft state
            
        Returns:
            dict: Control commands to apply to the aircraft
        """
        if not self.enabled:
            return {'aileron': 0.0}
        
        # Calculate lateral deviation from runway centerline
        current_lat = aircraft_state['latitude']
        current_lon = aircraft_state['longitude']
        
        self.lateral_deviation = self._calculate_lateral_deviation(
            current_lat, current_lon,
            self.runway_threshold_lat, self.runway_threshold_lon,
            self.runway_heading
        )
        
        # Error is the lateral deviation (target is 0)
        error = self.lateral_deviation
        
        # Compute PID control signal
        aileron = self.controller.compute(error)
        
        return {'aileron': aileron}


class Autoland:
    """
    Automatic landing system for aircraft.
    
    This class integrates multiple controllers to perform approach and landing
    operations, including glide slope, localizer, and speed control.
    
    Attributes:
        aircraft: The aircraft being controlled
        glide_slope (GlideSlope): Glide slope controller
        localizer (Localizer): Localizer controller
        speed_controller (SpeedHold): Airspeed controller
        target_approach_speed (float): Target approach speed in knots
        flare_altitude (float): Altitude to begin flare in feet AGL
        enabled (bool): Whether the autoland system is enabled
        phase (str): Current phase of landing (approach, flare, touchdown)
    """
    
    PHASES = ['approach', 'flare', 'touchdown', 'rollout']
    
    def __init__(
        self,
        aircraft,
        glide_slope: Optional[GlideSlope] = None,
        localizer: Optional[Localizer] = None,
        speed_controller: Optional[SpeedHold] = None,
        target_approach_speed: float = 120.0,
        target_glide_slope: float = 3.0,
        flare_altitude: float = 50.0
    ):
        """
        Initialize the autoland system.
        
        Args:
            aircraft: The aircraft being controlled
            glide_slope: Optional existing glide slope controller
            localizer: Optional existing localizer controller
            speed_controller: Optional existing speed controller
            target_approach_speed: Target approach speed in knots
            target_glide_slope: Target glide slope angle in degrees
            flare_altitude: Altitude to begin flare in feet AGL
        """
        self.aircraft = aircraft
        
        # Create or use provided controllers
        self.glide_slope = glide_slope or GlideSlope(target_glide_slope=target_glide_slope)
        self.localizer = localizer or Localizer()
        self.speed_controller = speed_controller or SpeedHold()
        
        self.target_approach_speed = target_approach_speed
        self.flare_altitude = flare_altitude
        
        self.enabled = False
        self.phase = 'approach'
        
        # For flare control
        self.flare_controller = PIDController(
            kp=0.03,
            ki=0.0,
            kd=0.2,
            output_limits=(-0.7, 0.3)
        )
        
        # For rollout
        self.rollout_controller = PIDController(
            kp=0.05,
            ki=0.001,
            kd=0.1,
            output_limits=(-0.5, 0.5)
        )
        
        # Touchdown detection
        self.touchdown_detected = False
        self.on_ground_counter = 0
    
    def set_runway(
        self,
        threshold_lat: float,
        threshold_lon: float,
        altitude: float,
        heading: float
    ) -> None:
        """
        Set the runway parameters for landing.
        
        Args:
            threshold_lat: Latitude of runway threshold in degrees
            threshold_lon: Longitude of runway threshold in degrees
            altitude: Runway elevation in feet
            heading: Runway heading in degrees
        """
        self.glide_slope.set_runway(threshold_lat, threshold_lon, altitude, heading)
        self.localizer.set_runway(threshold_lat, threshold_lon, heading)
    
    def enable(self) -> None:
        """Enable the autoland system."""
        self.enabled = True
        self.phase = 'approach'
        
        # Enable subsystems
        self.glide_slope.enable()
        self.localizer.enable()
        self.speed_controller.enable()
        
        # Set approach speed
        self.speed_controller.set_target(self.target_approach_speed)
        
        # Reset touchdown detection
        self.touchdown_detected = False
        self.on_ground_counter = 0
    
    def disable(self) -> None:
        """Disable the autoland system."""
        self.enabled = False
        
        # Disable subsystems
        self.glide_slope.disable()
        self.localizer.disable()
        self.speed_controller.disable()
    
    def _detect_touchdown(self, radar_altitude: float, vertical_speed: float) -> bool:
        """
        Detect touchdown based on radar altitude and vertical speed.
        
        Args:
            radar_altitude: Height above ground in feet
            vertical_speed: Vertical speed in feet per minute
            
        Returns:
            True if touchdown is detected, False otherwise
        """
        # Basic touchdown detection
        if radar_altitude < 5.0:
            self.on_ground_counter += 1
        else:
            self.on_ground_counter = 0
        
        # Require multiple consecutive on-ground readings to avoid false positives
        return self.on_ground_counter >= 3
    
    def _compute_flare(self, radar_altitude: float) -> float:
        """
        Compute elevator control for flare maneuver.
        
        Args:
            radar_altitude: Height above ground in feet
            
        Returns:
            Elevator control signal
        """
        # Exponential flare based on height above ground
        target_sink_rate = -100.0 * (radar_altitude / self.flare_altitude)
        current_sink_rate = self.aircraft.state['vertical_speed']
        
        # Compute error (target sink rate - current sink rate)
        error = target_sink_rate - current_sink_rate
        
        # Compute PID control signal
        elevator = self.flare_controller.compute(error)
        
        return elevator
    
    def _compute_rollout(self, heading: float, runway_heading: float) -> float:
        """
        Compute aileron control for rollout phase.
        
        Args:
            heading: Current aircraft heading in degrees
            runway_heading: Runway heading in degrees
            
        Returns:
            Aileron control signal
        """
        # Calculate heading error (normalized to ±180°)
        error = ((heading - runway_heading + 180) % 360) - 180
        
        # Compute PID control signal
        aileron = self.rollout_controller.compute(error)
        
        return aileron
    
    def update(self) -> Dict[str, float]:
        """
        Update the autoland system and compute control signals.
        
        Returns:
            Dictionary with control signals (elevator, aileron, throttle)
        """
        if not self.enabled:
            return {}
        
        # Get current aircraft state
        current_altitude = self.aircraft.state['altitude']
        radar_altitude = current_altitude - self.glide_slope.runway_altitude
        vertical_speed = self.aircraft.state['vertical_speed']
        heading = self.aircraft.state['heading']
        
        # Default control signals
        elevator = 0.0
        aileron = 0.0
        throttle = 0.0
        
        # Phase transitions
        if self.phase == 'approach' and radar_altitude < self.flare_altitude:
            self.phase = 'flare'
            print(f"Transitioning to flare phase at {radar_altitude:.1f} feet AGL")
        
        elif self.phase == 'flare' and self._detect_touchdown(radar_altitude, vertical_speed):
            self.phase = 'touchdown'
            self.touchdown_detected = True
            print("Touchdown detected")
        
        elif self.phase == 'touchdown' and self.aircraft.state['ground_speed'] < 60:
            self.phase = 'rollout'
            print("Transitioning to rollout phase")
        
        # Control based on current phase
        if self.phase == 'approach':
            elevator = self.glide_slope.update(self.aircraft)
            aileron = self.localizer.update(self.aircraft)
            throttle = self.speed_controller.update(self.aircraft)
        
        elif self.phase == 'flare':
            elevator = self._compute_flare(radar_altitude)
            aileron = self.localizer.update(self.aircraft)
            # Gradually reduce throttle during flare
            throttle_target = 0.2 * (radar_altitude / self.flare_altitude)
            self.speed_controller.set_target(self.target_approach_speed * 0.9)
            throttle = self.speed_controller.update(self.aircraft)
        
        elif self.phase == 'touchdown':
            elevator = 0.1  # Hold elevator slightly up
            aileron = self._compute_rollout(heading, self.glide_slope.runway_heading)
            throttle = 0.0  # Idle throttle
        
        elif self.phase == 'rollout':
            elevator = 0.1  # Hold elevator slightly up for aerodynamic braking
            aileron = self._compute_rollout(heading, self.glide_slope.runway_heading)
            throttle = 0.0  # Idle throttle
        
        # Apply controls to aircraft
        self.aircraft.set_control('elevator', elevator)
        self.aircraft.set_control('aileron', aileron)
        self.aircraft.set_control('throttle', throttle)
        
        return {
            'elevator': elevator,
            'aileron': aileron,
            'throttle': throttle
        }
    
    def get_status(self) -> Dict:
        """
        Get the current status of the autoland system.
        
        Returns:
            A dictionary with the current autoland status
        """
        return {
            'enabled': self.enabled,
            'phase': self.phase,
            'glide_slope': self.glide_slope.get_status(),
            'localizer': self.localizer.get_status(),
            'target_approach_speed': self.target_approach_speed,
            'current_speed': self.aircraft.state['airspeed'],
            'flare_altitude': self.flare_altitude,
            'radar_altitude': self.aircraft.state['altitude'] - self.glide_slope.runway_altitude,
            'touchdown_detected': self.touchdown_detected
        }
