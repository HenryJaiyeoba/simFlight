"""
Autopilot base module for aircraft control systems.

This module provides the base class for autopilot subsystems.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional

from ..controllers.pid_controller import PIDController


class AutopilotSubsystem(ABC):
    """
    Abstract base class for autopilot subsystems.
    
    This class defines the interface for all autopilot subsystems
    such as altitude hold, heading hold, etc.
    
    Attributes:
        name (str): Name of the autopilot subsystem
        controller (PIDController): PID controller for this subsystem
        enabled (bool): Whether the subsystem is currently enabled
        target (float): Target setpoint for the controlled variable
    """
    
    def __init__(
        self,
        name: str,
        kp: float = 0.0,
        ki: float = 0.0,
        kd: float = 0.0,
        output_limits: Optional[tuple] = None
    ):
        """
        Initialize the autopilot subsystem with the specified parameters.
        
        Args:
            name: Name of the autopilot subsystem
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: (min, max) tuple for output clamping
        """
        self.name = name
        self.controller = PIDController(
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=output_limits,
            enable_logging=True
        )
        self.enabled = False
        self.target = 0.0
    
    def enable(self) -> None:
        """Enable the autopilot subsystem."""
        self.enabled = True
        self.controller.reset()
    
    def disable(self) -> None:
        """Disable the autopilot subsystem."""
        self.enabled = False
    
    def set_target(self, target: float) -> None:
        """
        Set the target value for the autopilot subsystem.
        
        Args:
            target: Target setpoint
        """
        self.target = target
        self.controller.set_setpoint(target)
    
    def set_gains(self, kp: Optional[float] = None, ki: Optional[float] = None, kd: Optional[float] = None) -> None:
        """
        Update controller gain values.
        
        Args:
            kp: New proportional gain (None to keep current value)
            ki: New integral gain (None to keep current value)
            kd: New derivative gain (None to keep current value)
        """
        self.controller.set_gains(kp, ki, kd)
    
    def get_error(self) -> float:
        """
        Get the current error value (difference between setpoint and process variable).
        
        Returns:
            float: Current error value
        """
        return self.controller.last_error
    
    @abstractmethod
    def compute(self, aircraft_state: Dict[str, float]) -> Dict[str, float]:
        """
        Compute control commands based on current aircraft state.
        
        Args:
            aircraft_state: Current aircraft state
            
        Returns:
            dict: Control commands to apply to the aircraft
        """
        pass
