"""
Aircraft model module providing an abstract representation of an aircraft.

This module defines the Aircraft class which serves as a common interface
for both virtual and X-Plane-connected aircraft models.
"""

from typing import Dict, Any, Optional, Union
import numpy as np


class Aircraft:
    """
    Abstract aircraft model containing key flight state variables.
    
    This class represents an aircraft with its essential state variables and
    control inputs. It provides a consistent interface regardless of whether
    the backend is a virtual simulator or X-Plane.
    
    Attributes:
        simulator: The simulator backend (VirtualSimulator or XPlaneInterface)
        state (dict): Current aircraft state variables
        controls (dict): Current control input values
    """
    
    def __init__(self, simulator):
        """
        Initialize an aircraft model with the specified simulator backend.
        
        Args:
            simulator: The simulator backend (VirtualSimulator or XPlaneInterface)
        """
        self.simulator = simulator
        
        # Initialize aircraft state with default values
        self.state = {
            'altitude': 0.0,         # feet
            'vertical_speed': 0.0,   # feet per minute
            'pitch': 0.0,            # degrees
            'pitch_rate': 0.0,       # degrees per second
            'roll': 0.0,             # degrees
            'roll_rate': 0.0,        # degrees per second
            'heading': 0.0,          # degrees
            'heading_rate': 0.0,     # degrees per second
            'airspeed': 0.0,         # knots
            'ground_speed': 0.0,     # knots
            'latitude': 0.0,         # degrees
            'longitude': 0.0,        # degrees
            'throttle': 0.0,         # normalized [0-1]
            'elevator': 0.0,         # normalized [-1 to 1]
            'aileron': 0.0,          # normalized [-1 to 1]
            'rudder': 0.0,           # normalized [-1 to 1]
        }
        
        # Initialize control inputs
        self.controls = {
            'throttle': 0.0,         # normalized [0-1]
            'elevator': 0.0,         # normalized [-1 to 1]
            'aileron': 0.0,          # normalized [-1 to 1]
            'rudder': 0.0,           # normalized [-1 to 1]
        }
        
        # Get initial state from simulator
        self.get_state()
    
    def get_state(self) -> Dict[str, float]:
        """
        Get the current aircraft state from the simulator.
        
        Returns:
            dict: Current state variables including altitude, pitch, speed, heading, etc.
        """
        # Update state from simulator
        self.state = self.simulator.get_aircraft_state()
        return self.state
    
    def set_controls(self, 
                     throttle: Optional[float] = None, 
                     elevator: Optional[float] = None, 
                     aileron: Optional[float] = None, 
                     rudder: Optional[float] = None) -> None:
        """
        Set control inputs for the aircraft.
        
        Args:
            throttle: Throttle setting [0.0 to 1.0]
            elevator: Elevator deflection [-1.0 to 1.0], positive is nose up
            aileron: Aileron deflection [-1.0 to 1.0], positive is right roll
            rudder: Rudder deflection [-1.0 to 1.0], positive is yaw right
        """
        # Update only the controls that were provided
        if throttle is not None:
            self.controls['throttle'] = np.clip(throttle, 0.0, 1.0)
        
        if elevator is not None:
            self.controls['elevator'] = np.clip(elevator, -1.0, 1.0)
            
        if aileron is not None:
            self.controls['aileron'] = np.clip(aileron, -1.0, 1.0)
            
        if rudder is not None:
            self.controls['rudder'] = np.clip(rudder, -1.0, 1.0)
        
        # Send controls to simulator
        self.simulator.set_aircraft_controls(self.controls)
    
    def set_control(self, control_name: str, value: float) -> None:
        """
        Set a single control input for the aircraft.
        
        Args:
            control_name: Name of the control ('throttle', 'elevator', 'aileron', 'rudder')
            value: Control value
        """
        if control_name in self.controls:
            # Use the set_controls method to apply the control with proper clamping
            if control_name == 'throttle':
                self.set_controls(throttle=value)
            elif control_name == 'elevator':
                self.set_controls(elevator=value)
            elif control_name == 'aileron':
                self.set_controls(aileron=value)
            elif control_name == 'rudder':
                self.set_controls(rudder=value)
    
    def reset(self, 
              altitude: float = 5000.0, 
              airspeed: float = 120.0,
              heading: float = 0.0,
              position: Optional[Dict[str, float]] = None) -> None:
        """
        Reset the aircraft to a specified initial state.
        
        Args:
            altitude: Initial altitude in feet
            airspeed: Initial airspeed in knots
            heading: Initial heading in degrees
            position: Optional dictionary with latitude and longitude
        """
        initial_state = {
            'altitude': altitude,
            'airspeed': airspeed,
            'heading': heading
        }
        
        if position:
            initial_state['latitude'] = position.get('latitude', 0.0)
            initial_state['longitude'] = position.get('longitude', 0.0)
            
        self.simulator.reset_aircraft(initial_state)
        self.get_state()  # Update state from simulator after reset
        
    def __str__(self) -> str:
        """Return a string representation of the aircraft."""
        return (
            f"Aircraft(alt={self.state['altitude']:.1f}ft, "
            f"hdg={self.state['heading']:.1f}°, "
            f"spd={self.state['airspeed']:.1f}kt)"
        )
