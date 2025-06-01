"""
Altitude hold autopilot module for aircraft control systems.

This module provides an altitude hold autopilot subsystem that uses a PID
controller to maintain a target altitude.
"""

from typing import Dict

import numpy as np

from .base import AutopilotSubsystem


class AltitudeHold(AutopilotSubsystem):
    """
    Altitude hold autopilot subsystem.

    This subsystem uses a PID controller to maintain a target altitude by
    adjusting the elevator control.

    Attributes:
        vertical_speed_limit (float): Maximum vertical speed in feet per minute
        name (str): Name of the autopilot subsystem
        controller (PIDController): PID controller for altitude control
        enabled (bool): Whether the subsystem is currently enabled
        target (float): Target altitude in feet
    """

    def __init__(
        self,
        kp: float = 0.01,
        ki: float = 0.001,
        kd: float = 0.05,
        vertical_speed_limit: float = 1500.0,
    ):
        """
        Initialize the altitude hold autopilot with the specified parameters.

        Args:
            kp: Proportional gain for altitude error
            ki: Integral gain for altitude error
            kd: Derivative gain for altitude error
            vertical_speed_limit: Maximum vertical speed in feet per minute
        """
        # Initialize base class with appropriate output limits for elevator
        super().__init__(
            name="Altitude Hold",
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=(-0.5, 0.5),  # Elevator limits
        )

        self.vertical_speed_limit = vertical_speed_limit

    def compute(self, aircraft_state: Dict[str, float]) -> Dict[str, float]:
        """
        Compute elevator control to maintain the target altitude.

        Args:
            aircraft_state: Current aircraft state

        Returns:
            dict: Control commands with elevator value
        """
        if not self.enabled:
            return {"elevator": 0.0}

        # Get current altitude
        current_altitude = aircraft_state["altitude"]

        # Compute the elevator command using the PID controller
        elevator_command = self.controller.compute(current_altitude)

        # Apply vertical speed limiting if needed
        if "vertical_speed" in aircraft_state:
            vs = aircraft_state["vertical_speed"]

            # If climbing too fast, reduce elevator command
            if vs > self.vertical_speed_limit and elevator_command > 0:
                elevator_command *= self.vertical_speed_limit / vs

            # If descending too fast, increase elevator command
            elif vs < -self.vertical_speed_limit and elevator_command < 0:
                elevator_command *= self.vertical_speed_limit / abs(vs)

        return {"elevator": elevator_command}

    def update(self, aircraft) -> float:
        """
        Update the altitude hold controller and return elevator control.

        This method provides compatibility with the original API used by
        the waypoint navigation system.

        Args:
            aircraft: The aircraft being controlled

        Returns:
            float: Elevator control signal
        """
        # Get the result from compute method
        result = self.compute(aircraft.state)

        # Return just the elevator value
        return result["elevator"]
