#!/usr/bin/env python3
"""
Script to test the plotting functionality in the simFlight library.
"""

import numpy as np
import matplotlib.pyplot as plt
from simflight.simulators import VirtualSimulator
from simflight.aircraft import Aircraft
from simflight.autopilot import AltitudeHold, HeadingHold, SpeedHold
from simflight.visualization import plot_simulation_results, plot_pid_performance, interactive_plot

def test_plot_simulation_results():
    """Test the plot_simulation_results function with a simple simulation."""
    # Create a simulator
    initial_state = {
        'altitude': 5000.0,
        'airspeed': 120.0,
        'heading': 0.0,
        'pitch': 0.0,
        'roll': 0.0,
        'latitude': 37.0,
        'longitude': -122.0
    }
    simulator = VirtualSimulator(initial_state=initial_state, enable_logging=True)
    aircraft = Aircraft(simulator)
    
    # Create controllers
    altitude_controller = AltitudeHold()
    heading_controller = HeadingHold()
    speed_controller = SpeedHold()
    
    # Set targets
    target_altitude = 6000.0  # feet
    target_heading = 90.0     # degrees
    target_speed = 150.0      # knots
    
    altitude_controller.set_target(target_altitude)
    heading_controller.set_target(target_heading)
    speed_controller.set_target(target_speed)
    
    # Enable controllers
    altitude_controller.enable()
    heading_controller.enable()
    speed_controller.enable()
    
    # Run simulation for 120 seconds
    simulation_time = 120.0  # seconds
    dt = 0.1  # seconds
    steps = int(simulation_time / dt)
    
    print(f"Running simulation for {simulation_time} seconds ({steps} steps)...")
    for _ in range(steps):
        # Update aircraft state
        aircraft.get_state()
        
        # Calculate control outputs
        elevator = altitude_controller.update(aircraft)
        aileron = heading_controller.update(aircraft)
        throttle = speed_controller.update(aircraft)
        
        # Apply controls
        aircraft.set_controls(throttle=throttle, elevator=elevator, aileron=aileron)
        
        # Step simulation
        simulator.step(dt)
    
    # Get simulation history
    history = simulator.get_history()
    
    # Create setpoints dictionary
    setpoints = {
        'altitude': target_altitude,
        'heading': target_heading,
        'airspeed': target_speed
    }
    
    # Create control_history dictionary
    control_history = {
        'elevator': history['elevator'],
        'aileron': history['aileron'],
        'throttle': history['throttle']
    }
    
    # Plot results using matplotlib
    print("Plotting simulation results...")
    fig = plot_simulation_results(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history
    )
    
    # Save the figure
    fig.savefig('simulation_results.png')
    print("Saved plot to simulation_results.png")
    
    # Show the plot (comment this out if running in a headless environment)
    plt.show()

if __name__ == "__main__":
    test_plot_simulation_results()
