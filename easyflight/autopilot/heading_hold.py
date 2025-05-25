"""
Heading hold autopilot module for aircraft control systems.

This module provides a heading hold autopilot subsystem that uses a PID
controller to maintain a target heading.
"""

from typing import Dict
import numpy as np

from .base import AutopilotSubsystem


class HeadingHold(AutopilotSubsystem):
    """
    Heading hold autopilot subsystem.
    
    This subsystem uses a PID controller to maintain a target heading by
    adjusting the aileron control (bank angle).
    
    Attributes:
        max_bank_angle (float): Maximum bank angle in degrees
        name (str): Name of the autopilot subsystem
        controller (PIDController): PID controller for heading control
        enabled (bool): Whether the subsystem is currently enabled
        target (float): Target heading in degrees
    """
    
    def __init__(
        self,
        kp: float = 0.03,
        ki: float = 0.001,
        kd: float = 0.1,
        max_bank_angle: float = 25.0
    ):
        """
        Initialize the heading hold autopilot with the specified parameters.
        
        Args:
            kp: Proportional gain for heading error
            ki: Integral gain for heading error
            kd: Derivative gain for heading error
            max_bank_angle: Maximum bank angle in degrees
        """
        # Initialize base class with appropriate output limits for aileron
        super().__init__(
            name="Heading Hold",
            kp=kp,
            ki=ki,
            kd=kd,
            output_limits=(-0.5, 0.5)  # Aileron limits
        )
        
        self.max_bank_angle = max_bank_angle
    
    def compute(self, aircraft_state: Dict[str, float]) -> Dict[str, float]:
        """
        Compute aileron control to maintain the target heading.
        
        Args:
            aircraft_state: Current aircraft state
            
        Returns:
            dict: Control commands with aileron value
        """
        if not self.enabled:
            return {'aileron': 0.0}
        
        # Get current heading
        current_heading = aircraft_state['heading']
        
        # Calculate heading error (accounting for 0-360 wrap-around)
        error = ((self.target - current_heading + 180) % 360) - 180
        
        # Compute the aileron command using the PID controller with the error
        # Note: We don't use the standard compute method because of the special heading error calculation
        aileron_command = self.controller.kp * error
        
        # Add integral term
        self.controller.integral += error * self.controller.dt
        if self.controller.anti_windup and self.controller.anti_windup_limits:
            self.controller.integral = np.clip(
                self.controller.integral,
                self.controller.anti_windup_limits[0],
                self.controller.anti_windup_limits[1]
            )
        aileron_command += self.controller.ki * self.controller.integral
        
        # Add derivative term (if not the first computation)
        derivative = (error - self.last_error) / self.controller.dt
        aileron_command += self.controller.kd * derivative
        self.last_error = error
        
        # Apply output limits
        if self.controller.output_limits:
            aileron_command = np.clip(
                aileron_command,
                self.controller.output_limits[0],
                self.controller.output_limits[1]
            )
            
        # Update controller history
        if self.controller.enable_logging:
            current_time = self.controller.last_time
            self.controller.history['time'].append(current_time)
            self.controller.history['setpoint'].append(self.target)
            self.controller.history['process_variable'].append(current_heading)
            self.controller.history['error'].append(error)
            self.controller.history['p_term'].append(self.controller.kp * error)
            self.controller.history['i_term'].append(self.controller.ki * self.controller.integral)
            d_term = self.controller.kd * (error - self.last_error) / self.controller.dt if hasattr(self, 'last_error') else 0
            self.controller.history['d_term'].append(d_term)
            self.controller.history['output'].append(aileron_command)
        
        # Check current bank angle to prevent excessive banking
        if 'roll' in aircraft_state:
            current_bank = aircraft_state['roll']
            
            # If bank angle would exceed limits, reduce aileron command
            if (current_bank > self.max_bank_angle and aileron_command > 0) or \
               (current_bank < -self.max_bank_angle and aileron_command < 0):
                aileron_command = 0.0
        
        return {'aileron': aileron_command}
    
    def update(self, aircraft) -> float:
        """
        Update the heading hold controller and return aileron control.
        
        This method provides compatibility with the original API used by
        the waypoint navigation system.
        
        Args:
            aircraft: The aircraft being controlled
            
        Returns:
            float: Aileron control signal
        """
        # Get the result from compute method
        result = self.compute(aircraft.state)
        
        # Return just the aileron value
        return result['aileron']
