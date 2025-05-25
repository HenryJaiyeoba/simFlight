from typing import Dict

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
