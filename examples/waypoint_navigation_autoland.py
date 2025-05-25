"""
Waypoint Navigation and Autoland Example

This example demonstrates how to use the waypoint navigation and autoland
features of easyflight to create a complete flight path with an automatic landing.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from math import radians, degrees, sin, cos

from easyflight.simulators import VirtualSimulator
from easyflight.aircraft import Aircraft
from easyflight.autopilot import AltitudeHold, HeadingHold, SpeedHold, Autoland
from easyflight.navigation import Waypoint, FlightPlan, WaypointNavigation
from easyflight.visualization import plot_simulation_results, AutopilotDashboard


def main():
    """Run the waypoint navigation and autoland example."""
    # Create the simulator 
    initial_state = {
        'altitude': 1000.0,
        'airspeed': 120.0,
        'heading': 0.0,
        'pitch': 0.0,
        'roll': 0.0,
        'latitude': 37.505,
        'longitude': -122.300,
    }
    simulator = VirtualSimulator(initial_state=initial_state)
    aircraft = Aircraft(simulator)
    
    # Create autopilot controllers
    altitude_hold = AltitudeHold(kp=0.01, ki=0.001, kd=0.05)
    heading_hold = HeadingHold(kp=0.03, ki=0.001, kd=0.1)
    speed_hold = SpeedHold(kp=0.05, ki=0.01, kd=0.0)
    
    # Create waypoint navigation system
    nav = WaypointNavigation(
        aircraft,
        heading_controller=heading_hold,
        altitude_controller=altitude_hold,
        speed_controller=speed_hold
    )
    
    # Create a flight plan with waypoints
    flight_plan = FlightPlan(name="KSFO Approach")
    
    # Starting point (already at this position)
    flight_plan.add_waypoint(Waypoint(
        latitude=37.505,
        longitude=-122.300,
        altitude=1000.0,
        speed=120.0,
        name="START"
    ))
    
    # Turn to downwind
    flight_plan.add_waypoint(Waypoint(
        latitude=37.495,
        longitude=-122.285,
        altitude=3000.0,
        speed=120.0,
        name="DOWNWIND_ENTRY"
    ))
    
    # Downwind leg
    flight_plan.add_waypoint(Waypoint(
        latitude=37.470,
        longitude=-122.265,
        altitude=2500.0,
        speed=110.0,
        name="DOWNWIND"
    ))
    
    # Base leg turn
    flight_plan.add_waypoint(Waypoint(
        latitude=37.455,
        longitude=-122.285,
        altitude=2000.0,
        speed=100.0,
        name="BASE_TURN"
    ))
    
    # Final approach fix
    flight_plan.add_waypoint(Waypoint(
        latitude=37.470,
        longitude=-122.305,
        altitude=1500.0,
        speed=90.0,
        name="FINAL_APPROACH"
    ))
    
    # Runway threshold
    runway_threshold_lat = 37.485
    runway_threshold_lon = -122.325
    runway_altitude = 0.0
    runway_heading = 30.0
    
    flight_plan.add_waypoint(Waypoint(
        latitude=runway_threshold_lat,
        longitude=runway_threshold_lon,
        altitude=runway_altitude + 50.0,  # Slightly above runway
        speed=80.0,
        name="RUNWAY_THRESHOLD"
    ))
    
    # Set the flight plan in the navigation system
    nav.flight_plan = flight_plan
    
    # Create autoland system
    autoland = Autoland(
        aircraft,
        target_approach_speed=80.0,
        target_glide_slope=3.0,
        flare_altitude=30.0
    )
    
    # Set runway parameters for autoland
    autoland.set_runway(
        threshold_lat=runway_threshold_lat,
        threshold_lon=runway_threshold_lon,
        altitude=runway_altitude,
        heading=runway_heading
    )
    
    # Collect controllers for the dashboard
    controllers = {
        "altitude_hold": altitude_hold,
        "heading_hold": heading_hold,
        "speed_hold": speed_hold,
        "waypoint_navigation": nav,
        "autoland": autoland
    }
    
    # Create and start the dashboard
    dashboard = AutopilotDashboard(aircraft, controllers)
    
    # Start the navigation
    nav.enable()
    
    # This function will be called by the dashboard's update loop
    def simulation_step():
        """Run one step of the simulation."""
        # Update the navigation system
        nav.control_aircraft()
        
        # Check if we've reached the final waypoint
        if nav.flight_plan.current_waypoint_index == len(nav.flight_plan.waypoints) - 1:
            # Disable navigation and enable autoland for the final approach
            if not autoland.enabled:
                print("Reached final waypoint, activating autoland")
                nav.disable()
                autoland.enable()
        
        # If autoland is enabled, let it control the aircraft
        if autoland.enabled:
            autoland.update()
        
        # Run one step of the simulation
        simulator.step()
    
    # Register the simulation step with the dashboard
    dashboard.simulation_step = simulation_step
    
    # Start the dashboard (this will run the simulation)
    dashboard.start()
    
    # After the dashboard is closed, plot the results
    # print(simulator.history)
    fig = plot_simulation_results(time=simulator.history["time"], state_history=simulator.history)
    plt.show()

if __name__ == "__main__":
    main()
