#!/usr/bin/env python3
"""
Script to test the interactive plotting functionality in the easyflight library.
"""

import numpy as np
import plotly.io as pio
from easyflight.simulators import VirtualSimulator
from easyflight.aircraft import Aircraft
from easyflight.autopilot import AltitudeHold, HeadingHold, SpeedHold
from easyflight.visualization import interactive_plot

def test_interactive_plot():
    """Test the interactive_plot function with a simple simulation."""
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
    
    # Track controller history for plotting
    controller_history = {
        'AltitudeHold': {
            'time': [],
            'setpoint': [],
            'process_variable': [],
            'error': [],
            'p_term': [],
            'i_term': [],
            'd_term': [],
            'output': []
        }
    }
    
    print(f"Running simulation for {simulation_time} seconds ({steps} steps)...")
    for _ in range(steps):
        # Update aircraft state
        aircraft.get_state()
        
        # Calculate control outputs and update controller history
        elevator = altitude_controller.update(aircraft)
        aileron = heading_controller.update(aircraft)
        throttle = speed_controller.update(aircraft)
        
        # Record altitude controller history
        controller_history['AltitudeHold']['time'].append(simulator.time)
        controller_history['AltitudeHold']['setpoint'].append(altitude_controller.target)
        controller_history['AltitudeHold']['process_variable'].append(aircraft.state['altitude'])
        controller_history['AltitudeHold']['error'].append(altitude_controller.get_error())
        
        # Access PID terms directly from the controller
        controller_history['AltitudeHold']['p_term'].append(altitude_controller.controller.kp * altitude_controller.controller.last_error)
        controller_history['AltitudeHold']['i_term'].append(altitude_controller.controller.ki * altitude_controller.controller.integral)
        controller_history['AltitudeHold']['d_term'].append(0.0)  # Simplified, as we don't have direct access to d_term
        controller_history['AltitudeHold']['output'].append(elevator)
        
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
    
    # Plot results using plotly
    print("Plotting interactive simulation results...")
    fig = interactive_plot(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history,
        controller_history=controller_history,
        title="Interactive Flight Simulation Results"
    )
    
    # Save the figure as HTML
    pio.write_html(fig, 'interactive_simulation_results.html')
    print("Saved interactive plot to interactive_simulation_results.html")
    
    # Open the HTML file in the default browser
    import webbrowser
    webbrowser.open('interactive_simulation_results.html')

if __name__ == "__main__":
    test_interactive_plot()
