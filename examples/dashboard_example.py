"""
Real-time Dashboard Example

This example demonstrates how to use the AutopilotDashboard to visualize
and adjust autopilot parameters in real-time.
"""

import time
import numpy as np
import matplotlib.pyplot as plt

from simflight.simulators import VirtualSimulator
from simflight.aircraft import Aircraft
from simflight.autopilot import AltitudeHold, HeadingHold, SpeedHold
from simflight.visualization import AutopilotDashboard


def main():
    """Run the dashboard example."""
    # Create the simulator with initial altitude of 5000 feet
    simulator = VirtualSimulator()
    aircraft = Aircraft(simulator)
    
    # Create autopilot controllers
    altitude_hold = AltitudeHold(kp=0.01, ki=0.001, kd=0.05)
    heading_hold = HeadingHold(kp=0.03, ki=0.001, kd=0.1)
    speed_hold = SpeedHold(kp=0.05, ki=0.01, kd=0.0)
    
    # Set initial targets
    altitude_hold.set_target(6000.0)  # Target altitude of 6000 feet
    heading_hold.set_target(90.0)     # Target heading of 90 degrees (east)
    speed_hold.set_target(150.0)      # Target airspeed of 150 knots
    
    # Enable the controllers
    altitude_hold.enable()
    heading_hold.enable()
    speed_hold.enable()
    
    # Collect controllers for the dashboard
    controllers = {
        "altitude_hold": altitude_hold,
        "heading_hold": heading_hold,
        "speed_hold": speed_hold
    }
    
    # Create the dashboard
    dashboard = AutopilotDashboard(aircraft, controllers)
    
    # This function will be called by the dashboard's update loop
    def simulation_step():
        """Run one step of the simulation."""
        # Apply controller outputs to aircraft controls
        elevator = altitude_hold.update(aircraft)
        aileron = heading_hold.update(aircraft)
        throttle = speed_hold.update(aircraft)
        
        aircraft.set_control('elevator', elevator)
        aircraft.set_control('aileron', aileron)
        aircraft.set_control('throttle', throttle)
        
        # Run one step of the simulation
        simulator.step()
    
    # Register the simulation step with the dashboard
    dashboard.simulation_step = simulation_step
    
    # Start the dashboard (this will run the simulation)
    dashboard.start()
    
    # After the dashboard is closed, print a message
    print("Dashboard closed. Simulation ended.")


if __name__ == "__main__":
    main()
