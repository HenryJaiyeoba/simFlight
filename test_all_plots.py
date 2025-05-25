#!/usr/bin/env python3
"""
Script to demonstrate all graph generation capabilities in the easyflight library.
"""

import numpy as np
import matplotlib.pyplot as plt
import plotly.io as pio
from easyflight.simulators import VirtualSimulator
from easyflight.aircraft import Aircraft
from easyflight.autopilot import AltitudeHold, HeadingHold, SpeedHold
from easyflight.visualization import plot_simulation_results, plot_pid_performance, interactive_plot

def run_simulation():
    """Run a simulation and return the history data."""
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
        controller_history['AltitudeHold']['d_term'].append(0.0)  # Simplified
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
    
    return history, setpoints, control_history, controller_history


def test_all_plot_types():
    """Test all plotting functions in the easyflight library."""
    # Run simulation
    history, setpoints, control_history, controller_history = run_simulation()
    
    # 1. Test plot_simulation_results
    print("Testing plot_simulation_results...")
    fig1 = plot_simulation_results(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history
    )
    fig1.savefig('1_simulation_results.png')
    print("Saved plot to 1_simulation_results.png")
    
    # 2. Test plot_simulation_results with custom variables
    print("Testing plot_simulation_results with custom variables...")
    custom_variables = ['altitude', 'pitch', 'roll']
    fig2 = plot_simulation_results(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history,
        variables=custom_variables,
        fig_size=(10, 8)
    )
    fig2.savefig('2_custom_variables.png')
    print("Saved plot to 2_custom_variables.png")
    
    # 3. Test plot_pid_performance
    print("Testing plot_pid_performance...")
    fig3 = plot_pid_performance(
        controller_history=controller_history['AltitudeHold']
    )
    fig3.savefig('3_pid_performance.png')
    print("Saved plot to 3_pid_performance.png")
    
    # 4. Test interactive_plot
    print("Testing interactive_plot...")
    fig4 = interactive_plot(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history,
        title="Interactive Flight Simulation Results"
    )
    pio.write_html(fig4, '4_interactive_plot.html')
    print("Saved interactive plot to 4_interactive_plot.html")
    
    # 5. Test interactive_plot with controller history
    print("Testing interactive_plot with controller history...")
    fig5 = interactive_plot(
        time=history['time'],
        state_history=history,
        setpoints=setpoints,
        control_history=control_history,
        controller_history=controller_history,
        title="Interactive Flight Simulation with PID Performance"
    )
    pio.write_html(fig5, '5_interactive_with_pid.html')
    print("Saved interactive plot to 5_interactive_with_pid.html")
    
    # Display plots (comment this out if running in a headless environment)
    plt.show()


if __name__ == "__main__":
    test_all_plot_types()
