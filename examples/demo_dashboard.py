"""
Interactive Autopilot Dashboard Demo

This script demonstrates the GUI dashboard capabilities of simFlight,
allowing real-time visualization and parameter adjustment.

To run:
$ python demo_dashboard.py
"""

import numpy as np
import time
from simflight.simulators import VirtualSimulator
from simflight.aircraft import Aircraft
from simflight.autopilot import AltitudeHold, HeadingHold, SpeedHold, Autoland
from simflight.navigation import Waypoint, FlightPlan, WaypointNavigation
from simflight.visualization import AutopilotDashboard


def main():
    """Run the dashboard demonstration."""
    print("Starting simFlight Dashboard Demo...")
    print("This demo shows how to use the interactive dashboard for adjusting autopilot parameters.")
    
    # Create simulator with aircraft at 5000 feet
    initial_state = {
        'altitude': 5000.0,
        'airspeed': 120.0,
        'heading': 0.0,
        'pitch': 0.0,
        'roll': 0.0,
        'latitude': 37.0,
        'longitude': -122.0
    }
    
    simulator = VirtualSimulator(initial_state=initial_state)
    aircraft = Aircraft(simulator)
    
    # Create controllers with initial PID values
    altitude_hold = AltitudeHold(kp=0.01, ki=0.001, kd=0.05)
    heading_hold = HeadingHold(kp=0.03, ki=0.001, kd=0.1)
    speed_hold = SpeedHold(kp=0.05, ki=0.01, kd=0.0)
    
    # Set initial targets
    altitude_hold.set_target(6000.0)  # Climb to 6000 feet
    heading_hold.set_target(90.0)     # Turn to heading 090 (east)
    speed_hold.set_target(150.0)      # Accelerate to 150 knots
    
    # Enable the controllers
    altitude_hold.enable()
    heading_hold.enable()
    speed_hold.enable()
    
    # Create a simple flight plan for demonstration
    flight_plan = FlightPlan(name="Demo Plan")
    
    # Add a few waypoints
    flight_plan.add_waypoint(Waypoint(
        latitude=37.0,
        longitude=-122.0,
        altitude=5000.0,
        speed=120.0,
        name="START"
    ))
    
    flight_plan.add_waypoint(Waypoint(
        latitude=37.1,
        longitude=-121.9,
        altitude=6000.0,
        speed=150.0,
        name="WP1"
    ))
    
    flight_plan.add_waypoint(Waypoint(
        latitude=37.2,
        longitude=-121.8,
        altitude=7000.0,
        speed=160.0,
        name="WP2"
    ))
    
    flight_plan.add_waypoint(Waypoint(
        latitude=37.1,
        longitude=-121.7,
        altitude=6500.0,
        speed=140.0,
        name="WP3"
    ))
    
    # Create navigation system
    nav = WaypointNavigation(
        aircraft,
        flight_plan=flight_plan,
        heading_controller=heading_hold,
        altitude_controller=altitude_hold,
        speed_controller=speed_hold
    )
    
    # Set up autoland system
    autoland = Autoland(
        aircraft,
        target_approach_speed=90.0,
        target_glide_slope=3.0
    )
    
    # Set runway for demonstration
    autoland.set_runway(
        threshold_lat=37.0,
        threshold_lon=-121.5,
        altitude=0.0,
        heading=270.0
    )
    
    # Collect all controllers for the dashboard
    controllers = {
        "altitude_hold": altitude_hold,
        "heading_hold": heading_hold,
        "speed_hold": speed_hold,
        "waypoint_navigation": nav,
        "autoland": autoland
    }
    
    # Create the dashboard
    dashboard = AutopilotDashboard(aircraft, controllers)
    
    # Define the simulation step function
    def simulation_step():
        """Run one step of the simulation."""
        # If navigation is enabled, use it to control the aircraft
        if nav.enabled:
            nav.control_aircraft()
        # Otherwise, if autoland is enabled, use it
        elif autoland.enabled:
            autoland.update()
        # Otherwise, use the individual controllers
        else:
            elevator = altitude_hold.update(aircraft)
            aileron = heading_hold.update(aircraft)
            throttle = speed_hold.update(aircraft)
            
            aircraft.set_control('elevator', elevator)
            aircraft.set_control('aileron', aileron)
            aircraft.set_control('throttle', throttle)
        
        # Run the simulation
        simulator.step()
    
    # Set the simulation step function
    dashboard.simulation_step = simulation_step
    
    print("\nTHE DASHBOARD DIALOG IS OPENED:")
    print("Instructions:")
    print("1. Use the 'Controls' tab to adjust PID parameters")
    print("2. Use the 'Visualization' tab to monitor flight data")
    print("3. Use the 'Navigation' tab to manage waypoints and autoland")
    print("4. Close the window to end the simulation\n")
    
    # Start the dashboard (this will run the simulation)
    dashboard.start()
    
    print("Dashboard closed. Simulation ended.")


if __name__ == "__main__":
    main()
