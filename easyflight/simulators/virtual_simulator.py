"""
Virtual aircraft simulator module for educational purposes.

This module provides a simplified flight simulation environment that can be used
for educational purposes without requiring an external flight simulator.
"""

import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class VirtualSimulator:
    """
    Simple virtual aircraft simulator for educational purposes.

    This class implements a basic kinematic/dynamic model of aircraft motion
    for testing autopilot systems without requiring an external simulator.
    It provides a safe and fast environment for learning about flight controls.

    Attributes:
        state (dict): Current aircraft state variables
        controls (dict): Current control input values
        history (dict): Time history of state variables for analysis and plotting
        time (float): Current simulation time in seconds
        dt (float): Time step for simulation in seconds
    """

    def __init__(
        self,
        initial_state: Optional[Dict[str, float]] = None,
        enable_logging: bool = True,
    ):
        """
        Initialize the virtual simulator with the specified parameters.

        Args:
            initial_state: Optional dictionary with initial state values
            enable_logging: Whether to store simulation history
        """
        # Default initial state
        self.state = {
            "altitude": 5000.0,  # feet
            "vertical_speed": 0.0,  # feet per minute
            "pitch": 0.0,  # degrees
            "pitch_rate": 0.0,  # degrees per second
            "roll": 0.0,  # degrees
            "roll_rate": 0.0,  # degrees per second
            "heading": 0.0,  # degrees
            "heading_rate": 0.0,  # degrees per second
            "airspeed": 120.0,  # knots
            "ground_speed": 120.0,  # knots
            "latitude": 37.0,  # degrees
            "longitude": -122.0,  # degrees
            "throttle": 0.5,  # normalized [0-1]
            "elevator": 0.0,  # normalized [-1 to 1]
            "aileron": 0.0,  # normalized [-1 to 1]
            "rudder": 0.0,  # normalized [-1 to 1]
        }

        # Override defaults with provided initial state
        if initial_state:
            for key, value in initial_state.items():
                if key in self.state:
                    self.state[key] = value

        # Current control inputs
        self.controls = {
            "throttle": self.state["throttle"],
            "elevator": self.state["elevator"],
            "aileron": self.state["aileron"],
            "rudder": self.state["rudder"],
        }

        # Simulation parameters
        self.time = 0.0
        self.dt = 0.1  # Default time step in seconds

        # Aircraft model parameters (simplified)
        self.parameters = {
            # Throttle to airspeed relationship
            "throttle_to_speed": 200.0,  # Max speed at full throttle (knots)
            "throttle_response": 0.05,  # Speed response to throttle (per second)
            # Elevator effectiveness
            "elevator_to_pitch_rate": 8.0,  # Pitch rate per full elevator (deg/s)
            "pitch_damping": 1.5,  # Natural pitch stability
            # Aileron effectiveness
            "aileron_to_roll_rate": 20.0,  # Roll rate per full aileron (deg/s)
            "roll_damping": 3.0,  # Natural roll stability
            # Rudder effectiveness
            "rudder_to_heading_rate": 3.0,  # Direct heading rate from rudder (deg/s)
            "roll_to_heading_rate": 0.1,  # Turn rate from bank angle (deg/s per deg)
            # Pitch to vertical speed relationship
            "pitch_to_vs": 400.0,  # Vertical speed per degree of pitch (ft/min)
            # Speed to distance traveled
            "speed_to_distance": 1.0 / 60.0,  # Nautical miles per knot per minute
            # Earth model
            "earth_radius": 3440.0,  # Earth radius in nautical miles
        }

        # Initialize history logging
        self.enable_logging = enable_logging
        if enable_logging:
            self.history = {key: [value] for key, value in self.state.items()}
            self.history["time"] = [self.time]

    def set_aircraft_controls(self, controls: Dict[str, float]) -> None:
        """
        Set control inputs for the simulated aircraft.

        Args:
            controls: Dictionary containing control values (throttle, elevator, aileron, rudder)
        """
        for control, value in controls.items():
            if control in self.controls:
                self.controls[control] = value

    def get_aircraft_state(self) -> Dict[str, float]:
        """
        Get the current state of the simulated aircraft.

        Returns:
            dict: Current state variables
        """
        return self.state.copy()

    def reset_aircraft(self, initial_state: Dict[str, float]) -> None:
        """
        Reset the aircraft to a specified initial state.

        Args:
            initial_state: Dictionary with initial state values
        """
        for key, value in initial_state.items():
            if key in self.state:
                self.state[key] = value

        # Reset control inputs
        self.controls = {
            "throttle": initial_state.get("throttle", 0.5),
            "elevator": initial_state.get("elevator", 0.0),
            "aileron": initial_state.get("aileron", 0.0),
            "rudder": initial_state.get("rudder", 0.0),
        }

        # Reset simulation time
        self.time = 0.0

        # Reset history if logging is enabled
        if self.enable_logging:
            self.history = {key: [value] for key, value in self.state.items()}
            self.history["time"] = [self.time]

    def step(self, dt: Optional[float] = None) -> Dict[str, float]:
        """
        Advance the simulation by one time step.

        Args:
            dt: Time step in seconds (default: self.dt)

        Returns:
            dict: Updated aircraft state
        """
        # Use provided dt or default
        dt = dt if dt is not None else self.dt

        # Store the current state for calculations
        current_state = self.state.copy()

        # Calculate airspeed based on throttle setting
        target_airspeed = (
            self.parameters["throttle_to_speed"] * self.controls["throttle"]
        )
        airspeed_change = (
            (target_airspeed - current_state["airspeed"])
            * self.parameters["throttle_response"]
            * dt
        )
        new_airspeed = current_state["airspeed"] + airspeed_change

        # Calculate pitch rate based on elevator input and current pitch
        elevator_pitch_rate = (
            self.parameters["elevator_to_pitch_rate"] * self.controls["elevator"]
        )
        damping_pitch_rate = -self.parameters["pitch_damping"] * current_state["pitch"]
        new_pitch_rate = elevator_pitch_rate + damping_pitch_rate

        # Calculate new pitch
        new_pitch = current_state["pitch"] + new_pitch_rate * dt

        # Calculate roll rate based on aileron input and roll damping
        aileron_roll_rate = (
            self.parameters["aileron_to_roll_rate"] * self.controls["aileron"]
        )
        damping_roll_rate = -self.parameters["roll_damping"] * current_state["roll"]
        new_roll_rate = aileron_roll_rate + damping_roll_rate

        # Calculate new roll
        new_roll = current_state["roll"] + new_roll_rate * dt

        # Calculate heading rate based on rudder input, roll angle
        rudder_heading_rate = (
            self.parameters["rudder_to_heading_rate"] * self.controls["rudder"]
        )
        roll_heading_rate = self.parameters["roll_to_heading_rate"] * new_roll
        new_heading_rate = rudder_heading_rate + roll_heading_rate

        # Calculate new heading (0-360 degrees)
        new_heading = (current_state["heading"] + new_heading_rate * dt) % 360.0

        # Calculate vertical speed based on pitch angle
        new_vertical_speed = new_pitch * self.parameters["pitch_to_vs"]

        # Calculate new altitude
        altitude_change = new_vertical_speed * (dt / 60.0)  # Convert ft/min to ft/s
        new_altitude = current_state["altitude"] + altitude_change

        # Calculate ground speed (simplified - same as airspeed)
        new_ground_speed = new_airspeed

        # Calculate new position (lat/lon) based on heading and ground speed
        distance = new_ground_speed * self.parameters["speed_to_distance"] * dt
        heading_rad = np.radians(new_heading)

        # Simple approximations
        lat_change = distance * np.cos(heading_rad) / 60.0  # 1 minute of lat = 1 nm
        lon_change = (
            distance
            * np.sin(heading_rad)
            / (60.0 * np.cos(np.radians(current_state["latitude"])))
        )

        new_latitude = current_state["latitude"] + lat_change
        new_longitude = current_state["longitude"] + lon_change

        # Update state
        self.state.update(
            {
                "altitude": new_altitude,
                "vertical_speed": new_vertical_speed,
                "pitch": new_pitch,
                "pitch_rate": new_pitch_rate,
                "roll": new_roll,
                "roll_rate": new_roll_rate,
                "heading": new_heading,
                "heading_rate": new_heading_rate,
                "airspeed": new_airspeed,
                "ground_speed": new_ground_speed,
                "latitude": new_latitude,
                "longitude": new_longitude,
                "throttle": self.controls["throttle"],
                "elevator": self.controls["elevator"],
                "aileron": self.controls["aileron"],
                "rudder": self.controls["rudder"],
            }
        )

        # Update simulation time
        self.time += dt

        # Log state if enabled
        if self.enable_logging:
            for key, value in self.state.items():
                self.history[key].append(value)
            self.history["time"].append(self.time)

        return self.state

    def get_history(self) -> Dict[str, List[float]]:
        """
        Get the simulation history for analysis and visualization.

        Returns:
            dict: Time history of state variables
        """
        if not self.enable_logging:
            raise ValueError("Logging is disabled for this simulator")
        return self.history
