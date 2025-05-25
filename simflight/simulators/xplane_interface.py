"""
X-Plane simulator interface module using X-Plane Connect (XPC) plugin.

This module provides an interface to connect to the X-Plane flight simulator
using the X-Plane Connect (XPC) plugin, allowing real-time control and data
acquisition from X-Plane.
"""

import socket
import struct
import time
import numpy as np
from typing import Dict, Any, List, Optional, Tuple, Union


class XPlaneInterface:
    """
    Interface with the X-Plane simulator using the X-Plane Connect (XPC) plugin.
    
    This class provides methods to connect to X-Plane, read flight state data,
    and send control inputs to the simulator in real-time.
    
    Attributes:
        host (str): X-Plane host address
        port (int): X-Plane Connect plugin port
        socket: Socket connection to X-Plane
        connected (bool): Connection status
        timeout (float): Socket timeout in seconds
        enable_logging (bool): Whether to store history
        history (dict): Time history of state variables
    """
    
    # X-Plane Connect data reference definitions
    DATA_REFS = {
        'altitude': 'sim/flightmodel/position/elevation',        # meters MSL
        'altitude_agl': 'sim/flightmodel/position/y_agl',        # meters AGL
        'pitch': 'sim/flightmodel/position/theta',               # degrees
        'roll': 'sim/flightmodel/position/phi',                  # degrees
        'heading': 'sim/flightmodel/position/psi',               # degrees
        'airspeed': 'sim/flightmodel/position/indicated_airspeed', # meters/sec
        'vertical_speed': 'sim/flightmodel/position/vh_ind',      # meters/sec
        'latitude': 'sim/flightmodel/position/latitude',         # degrees
        'longitude': 'sim/flightmodel/position/longitude',       # degrees
        'throttle': 'sim/flightmodel/engine/ENGN_thro',          # 0-1
        'elevator': 'sim/flightmodel/controls/elv1_def',         # degrees
        'aileron': 'sim/flightmodel/controls/ail1_def',          # degrees
        'rudder': 'sim/flightmodel/controls/rud1_def',           # degrees
        'pitch_rate': 'sim/flightmodel/position/Q',              # rad/s
        'roll_rate': 'sim/flightmodel/position/P',               # rad/s
        'heading_rate': 'sim/flightmodel/position/R',            # rad/s
    }
    
    # Control inputs
    CONTROL_REFS = {
        'throttle': 'sim/multiplayer/controls/engine_throttle_request',  # [0, 1]
        'elevator': 'sim/multiplayer/controls/yoke_pitch_ratio',        # [-1, 1]
        'aileron': 'sim/multiplayer/controls/yoke_roll_ratio',          # [-1, 1]
        'rudder': 'sim/multiplayer/controls/yoke_heading_ratio',        # [-1, 1]
    }
    
    # Message types
    DREF = b"DREF"
    RREF = b"RREF"
    
    def __init__(
        self,
        host: str = '127.0.0.1',
        port: int = 49009,
        timeout: float = 1.0,
        enable_logging: bool = True
    ):
        """
        Initialize the X-Plane interface with the specified parameters.
        
        Args:
            host: X-Plane host address (default: localhost)
            port: X-Plane Connect plugin port (default: 49009)
            timeout: Socket timeout in seconds
            enable_logging: Whether to store state history
        """
        self.host = host
        self.port = port
        self.socket = None
        self.connected = False
        self.timeout = timeout
        self.enable_logging = enable_logging
        self.history = {} if enable_logging else None
        
        # Set up unit conversions
        self.units_conversion = {
            'altitude': 3.28084,       # m to ft
            'altitude_agl': 3.28084,   # m to ft
            'airspeed': 1.94384,       # m/s to knots
            'vertical_speed': 196.85,  # m/s to ft/min
            'pitch_rate': 57.2958,     # rad/s to deg/s
            'roll_rate': 57.2958,      # rad/s to deg/s
            'heading_rate': 57.2958,   # rad/s to deg/s
        }
        
        # Current aircraft state (with default values)
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
        
        # Try to connect to X-Plane
        try:
            self.connect()
        except (socket.error, ConnectionRefusedError) as e:
            print(f"Warning: Could not connect to X-Plane: {e}")
            print("Make sure X-Plane is running with the XPC plugin installed.")
            print("The interface will retry connecting when methods are called.")
    
    def connect(self) -> bool:
        """
        Connect to the X-Plane simulator.
        
        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.timeout)
            self.connected = True
            
            # Initialize history if logging is enabled
            if self.enable_logging:
                self.history = {key: [] for key in self.state.keys()}
                self.history['time'] = []
            
            return True
            
        except (socket.error, ConnectionRefusedError) as e:
            self.connected = False
            return False
    
    def disconnect(self) -> None:
        """Close the connection to X-Plane."""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
    
    def _ensure_connection(self) -> bool:
        """
        Ensure that there is a valid connection to X-Plane.
        
        Returns:
            bool: True if connected, False if connection failed
        """
        if not self.connected or not self.socket:
            return self.connect()
        return True
    
    def _send_data(self, data: bytes) -> None:
        """
        Send data to X-Plane.
        
        Args:
            data: Bytes to send
        """
        if not self._ensure_connection():
            raise ConnectionError("Not connected to X-Plane")
            
        try:
            self.socket.sendto(data, (self.host, self.port))
        except socket.error as e:
            self.connected = False
            raise ConnectionError(f"Error sending data to X-Plane: {e}")
    
    def _receive_data(self, buffer_size: int = 8192) -> bytes:
        """
        Receive data from X-Plane.
        
        Args:
            buffer_size: Size of the receive buffer
            
        Returns:
            bytes: Received data
        """
        if not self._ensure_connection():
            raise ConnectionError("Not connected to X-Plane")
            
        try:
            data, addr = self.socket.recvfrom(buffer_size)
            return data
        except socket.timeout:
            raise TimeoutError("Timeout waiting for data from X-Plane")
        except socket.error as e:
            self.connected = False
            raise ConnectionError(f"Error receiving data from X-Plane: {e}")
    
    def _read_dataref(self, dataref: str, freq: int = 1) -> float:
        """
        Read a single dataref value from X-Plane.
        
        Args:
            dataref: X-Plane dataref to read
            freq: Frequency for continuous ref updates
            
        Returns:
            float: Value of the dataref
        """
        message = struct.pack("<5sii400s", self.RREF, freq, 0, dataref.encode())
        self._send_data(message)
        data = self._receive_data()
        
        # Parse the result (header 5 bytes, then 8 bytes per dataref)
        if len(data) < 13:  # 5 + 8
            raise ValueError("Received data too short")
            
        # Extract the value from the response
        value = struct.unpack("<5sii", data[:13])[2]
        return float(value)
    
    def _read_datarefs(self, datarefs: List[str], freq: int = 1) -> Dict[str, float]:
        """
        Read multiple dataref values from X-Plane in a single request.
        
        Args:
            datarefs: List of X-Plane datarefs to read
            freq: Frequency for continuous ref updates
            
        Returns:
            dict: Dictionary mapping dataref names to values
        """
        # Build the request message
        message = struct.pack("<5si", self.RREF, freq)
        
        # Add each dataref to the request
        for i, dataref in enumerate(datarefs):
            message += struct.pack("<i400s", i, dataref.encode())
        
        # Send the request
        self._send_data(message)
        
        # Receive the response
        data = self._receive_data()
        
        # Parse the result (header 5 bytes, then 8 bytes per dataref)
        if len(data) < 5 + 8 * len(datarefs):
            raise ValueError("Received data too short")
            
        # Extract the values from the response
        result = {}
        for i, dataref in enumerate(datarefs):
            index, value = struct.unpack("<if", data[5 + 8*i:5 + 8*(i+1)])
            result[dataref] = value
            
        return result
    
    def _set_dataref(self, dataref: str, value: float) -> None:
        """
        Set a dataref value in X-Plane.
        
        Args:
            dataref: X-Plane dataref to set
            value: Value to set
        """
        message = struct.pack("<5sif400s", self.DREF, 1, value, dataref.encode())
        self._send_data(message)
    
    def get_aircraft_state(self) -> Dict[str, float]:
        """
        Get the current state of the aircraft from X-Plane.
        
        Returns:
            dict: Current state variables
        """
        # Try to read all the required datarefs
        try:
            # Prepare list of datarefs to read
            datarefs = list(self.DATA_REFS.values())
            
            # Read the values from X-Plane
            values = self._read_datarefs(datarefs)
            
            # Map the values to our state dictionary with proper unit conversion
            for state_key, dataref in self.DATA_REFS.items():
                # Get the raw value
                raw_value = values[dataref]
                
                # Apply unit conversion if needed
                if state_key in self.units_conversion:
                    raw_value *= self.units_conversion[state_key]
                
                # Update the state
                self.state[state_key] = raw_value
            
            # Ground speed is the same as airspeed for simplicity
            self.state['ground_speed'] = self.state['airspeed']
            
            # Log state if enabled
            if self.enable_logging:
                current_time = time.time()
                for key, value in self.state.items():
                    self.history[key].append(value)
                self.history['time'].append(current_time)
                
            return self.state
            
        except (ConnectionError, TimeoutError) as e:
            print(f"Error getting aircraft state: {e}")
            return self.state
    
    def set_aircraft_controls(self, controls: Dict[str, float]) -> None:
        """
        Set control inputs for the aircraft in X-Plane.
        
        Args:
            controls: Dictionary containing control values (throttle, elevator, aileron, rudder)
        """
        try:
            # Send each control to X-Plane
            for control, value in controls.items():
                if control in self.CONTROL_REFS:
                    # Update internal state
                    self.state[control] = value
                    
                    # Send to X-Plane
                    self._set_dataref(self.CONTROL_REFS[control], value)
                    
        except (ConnectionError, TimeoutError) as e:
            print(f"Error setting aircraft controls: {e}")
    
    def reset_aircraft(self, initial_state: Dict[str, float]) -> None:
        """
        Reset the aircraft to a specified initial state in X-Plane.
        
        Note: X-Plane does not provide a direct API for resetting aircraft state.
        This method uses available dataref operations to approximate a reset.
        
        Args:
            initial_state: Dictionary with initial state values
        """
        try:
            # Set position (latitude, longitude, altitude)
            if 'latitude' in initial_state:
                self._set_dataref("sim/flightmodel/position/latitude", initial_state['latitude'])
                
            if 'longitude' in initial_state:
                self._set_dataref("sim/flightmodel/position/longitude", initial_state['longitude'])
                
            if 'altitude' in initial_state:
                # Convert feet to meters
                altitude_m = initial_state['altitude'] / 3.28084
                self._set_dataref("sim/flightmodel/position/elevation", altitude_m)
                
            # Set orientation (heading, pitch, roll)
            if 'heading' in initial_state:
                self._set_dataref("sim/flightmodel/position/psi", initial_state['heading'])
                
            if 'pitch' in initial_state:
                self._set_dataref("sim/flightmodel/position/theta", initial_state['pitch'])
                
            if 'roll' in initial_state:
                self._set_dataref("sim/flightmodel/position/phi", initial_state['roll'])
                
            # Set airspeed
            if 'airspeed' in initial_state:
                # Convert knots to m/s
                airspeed_ms = initial_state['airspeed'] / 1.94384
                self._set_dataref("sim/flightmodel/position/indicated_airspeed", airspeed_ms)
                
            # Update our internal state
            self.get_aircraft_state()
            
        except (ConnectionError, TimeoutError) as e:
            print(f"Error resetting aircraft: {e}")
    
    def get_history(self) -> Dict[str, List[float]]:
        """
        Get the history of aircraft state for analysis and visualization.
        
        Returns:
            dict: Time history of state variables
        """
        if not self.enable_logging:
            raise ValueError("Logging is disabled for this interface")
        return self.history
