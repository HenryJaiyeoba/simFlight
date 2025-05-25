# simFlight: Aircraft Autopilot Control Systems Library

An educational Python library for teaching undergraduate students the principles of aircraft autopilot control systems with a focus on PID controllers.

## Overview

simFlight is designed to provide a beginner-friendly, scalable, and educational platform for learning about aircraft control systems. It features both virtual simulation capabilities and integration with X-Plane through the X-Plane Connect plugin.

## Features

- **PID Controller Implementation**: Core PID control algorithm with anti-windup and output clamping
- **Virtual Flight Simulator**: Simple kinematic/dynamic models for offline learning
- **X-Plane Integration**: Real-time connection to X-Plane simulator
- **Autopilot Subsystems**: Modular components for altitude, heading, and speed control
- **Waypoint Navigation**: Navigate between geographical waypoints with altitude and speed constraints
- **Autoland System**: Automatic approach and landing capabilities with glide slope and localizer tracking
- **Real-time GUI Dashboard**: Interactive dashboard for parameter tuning and flight visualization
- **Visualization Tools**: Interactive plots for monitoring system performance
- **Jupyter Notebooks**: Step-by-step tutorials explaining control theory and implementation
- **Comprehensive Testing**: Unit and integration tests for all components

## Getting Started

### Installation

```bash
pip install simflight
```

### Quick Example

```python
from simflight.controllers import PIDController
from simflight.simulators import VirtualSimulator
from simflight.aircraft import Aircraft
from simflight.autopilot import AltitudeHold
import matplotlib.pyplot as plt

# Create a virtual simulator and aircraft
simulator = VirtualSimulator()
aircraft = Aircraft(simulator)

# Create an altitude hold autopilot system
altitude_controller = AltitudeHold(kp=0.01, ki=0.001, kd=0.05)

# Run a simulation
target_altitude = 5000  # feet
simulation_time = 120   # seconds
dt = 0.1                # seconds

time_points = []
actual_altitude = []
control_signals = []

# Simulation loop
for t in range(int(simulation_time / dt)):
    current_time = t * dt

    # Get current aircraft state
    state = aircraft.get_state()

    # Compute control command
    elevator_command = altitude_controller.compute(state, target_altitude)

    # Apply control to aircraft
    aircraft.set_controls(elevator=elevator_command)

    # Advance simulation
    simulator.step(dt)

    # Record data for plotting
    time_points.append(current_time)
    actual_altitude.append(state['altitude'])
    control_signals.append(elevator_command)

# Plot results
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(time_points, actual_altitude, 'b-', label='Actual Altitude')
plt.plot(time_points, [target_altitude] * len(time_points), 'r--', label='Target Altitude')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (ft)')
plt.legend()

plt.subplot(2, 1, 2)
plt.plot(time_points, control_signals, 'g-', label='Elevator Command')
plt.xlabel('Time (s)')
plt.ylabel('Control Signal')
plt.legend()

plt.tight_layout()
plt.show()
```

## Documentation

For detailed documentation and examples, see the [documentation site](https://simflight.readthedocs.io).

### Tutorials

The library comes with several Jupyter notebooks that provide step-by-step tutorials:

1. **Introduction to PID Controllers** - Learn the basics of PID control theory
2. **Altitude Hold Autopilot** - Implement a basic altitude hold system
3. **Waypoint Navigation and Autoland** - Create a complete flight management system

### Examples

Several example scripts demonstrate key features:

- **Basic PID Control** - Simple demonstration of PID controllers
- **Waypoint Navigation** - Navigate between a series of waypoints
- **Autoland System** - Automatic approach and landing
- **Real-time Dashboard** - GUI for parameter adjustment and visualization

## Project Structure

```
simflight/
├── simflight/           # Main package
│   ├── __init__.py
│   ├── controllers/      # PID and other controllers
│   ├── aircraft/         # Aircraft models
│   ├── autopilot/        # Autopilot subsystems
│   ├── simulators/       # Virtual and X-Plane simulators
│   ├── visualization/    # Plotting and visualization tools
│   └── utils/            # Utility functions
├── tests/                # Unit and integration tests
├── notebooks/            # Jupyter notebooks
├── docs/                 # Documentation
├── examples/             # Example scripts
├── setup.py              # Package setup script
└── README.md             # Project readme
```

## Prerequisites

- Python 3.8+
- For X-Plane integration: X-Plane 11 with X-Plane Connect plugin installed

## License

MIT License
