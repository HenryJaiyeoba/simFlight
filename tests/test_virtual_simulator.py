"""
Tests for the VirtualSimulator class.
"""

import pytest
import numpy as np
from simflight.simulators import VirtualSimulator


def test_virtual_simulator_initialization():
    """Test VirtualSimulator initialization with default values."""
    sim = VirtualSimulator()
    assert sim.state['altitude'] == 5000.0
    assert sim.state['airspeed'] == 120.0
    assert sim.state['heading'] == 0.0
    assert sim.state['pitch'] == 0.0
    assert sim.state['roll'] == 0.0
    assert sim.time == 0.0
    assert sim.dt == 0.1
    assert sim.enable_logging is True


def test_virtual_simulator_initialization_with_params():
    """Test VirtualSimulator initialization with custom values."""
    initial_state = {
        'altitude': 10000.0,
        'airspeed': 150.0,
        'heading': 90.0
    }
    sim = VirtualSimulator(initial_state=initial_state, enable_logging=False)
    assert sim.state['altitude'] == 10000.0
    assert sim.state['airspeed'] == 150.0
    assert sim.state['heading'] == 90.0
    assert sim.enable_logging is False


def test_virtual_simulator_step():
    """Test stepping the simulator forward in time."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Run a single step with default time
    initial_altitude = sim.state['altitude']
    sim.step()
    assert sim.time == 0.1
    
    # Run a step with custom time
    sim.step(dt=0.5)
    assert sim.time == 0.6


def test_virtual_simulator_control_inputs():
    """Test setting control inputs."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set controls
    controls = {
        'throttle': 0.8,
        'elevator': 0.2,
        'aileron': -0.1,
        'rudder': 0.0
    }
    sim.set_aircraft_controls(controls)
    
    # Check that controls were set
    assert sim.controls['throttle'] == 0.8
    assert sim.controls['elevator'] == 0.2
    assert sim.controls['aileron'] == -0.1
    assert sim.controls['rudder'] == 0.0


def test_virtual_simulator_throttle_airspeed():
    """Test that throttle affects airspeed."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set initial state
    initial_airspeed = sim.state['airspeed']
    
    # Set throttle to max
    sim.set_aircraft_controls({'throttle': 1.0})
    
    # Run simulation for a while
    for _ in range(20):
        sim.step(dt=0.5)
    
    # Check that airspeed increased
    assert sim.state['airspeed'] > initial_airspeed


def test_virtual_simulator_elevator_pitch():
    """Test that elevator affects pitch."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set initial state
    initial_pitch = sim.state['pitch']
    
    # Pull back on elevator (positive elevator = nose up)
    sim.set_aircraft_controls({'elevator': 0.5})
    
    # Run simulation for a while
    for _ in range(10):
        sim.step(dt=0.1)
    
    # Check that pitch increased (nose up)
    assert sim.state['pitch'] > initial_pitch
    
    # Push forward on elevator (negative elevator = nose down)
    sim.set_aircraft_controls({'elevator': -0.5})
    
    # Run simulation for a while
    for _ in range(20):
        sim.step(dt=0.1)
    
    # Check that pitch decreased (nose down)
    assert sim.state['pitch'] < initial_pitch


def test_virtual_simulator_aileron_roll():
    """Test that aileron affects roll."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set initial state
    initial_roll = sim.state['roll']
    
    # Apply right roll
    sim.set_aircraft_controls({'aileron': 0.5})
    
    # Run simulation for a while
    for _ in range(10):
        sim.step(dt=0.1)
    
    # Check that roll increased (right wing down)
    assert sim.state['roll'] > initial_roll
    
    # Apply left roll
    sim.set_aircraft_controls({'aileron': -0.5})
    
    # Run simulation for a while
    for _ in range(20):
        sim.step(dt=0.1)
    
    # Check that roll decreased (left wing down)
    assert sim.state['roll'] < initial_roll


def test_virtual_simulator_pitch_altitude():
    """Test that pitch affects altitude."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set initial state
    initial_altitude = sim.state['altitude']
    
    # Pull back on elevator to increase pitch
    sim.set_aircraft_controls({'elevator': 0.5})
    
    # Run simulation for a while
    for _ in range(20):
        sim.step(dt=0.5)
    
    # Check that altitude increased
    assert sim.state['altitude'] > initial_altitude
    
    # Push forward on elevator to decrease pitch
    sim.set_aircraft_controls({'elevator': -0.5})
    
    # Run simulation for a while
    for _ in range(40):
        sim.step(dt=0.5)
    
    # Check that altitude decreased
    assert sim.state['altitude'] < initial_altitude


def test_virtual_simulator_bank_turn():
    """Test that bank angle affects heading."""
    sim = VirtualSimulator(enable_logging=False)
    
    # Set initial state
    initial_heading = sim.state['heading']
    
    # Bank right
    sim.set_aircraft_controls({'aileron': 0.5})
    
    # Run simulation for a while
    for _ in range(20):
        sim.step(dt=0.5)
    
    # Check that heading increased (turn right)
    # Using modular arithmetic for 0-360 degrees
    heading_change = (sim.state['heading'] - initial_heading) % 360
    assert heading_change > 0 and heading_change < 180


def test_virtual_simulator_reset():
    """Test resetting the simulator."""
    sim = VirtualSimulator()
    
    # Change simulator state
    sim.set_aircraft_controls({'throttle': 1.0, 'elevator': 0.5})
    for _ in range(10):
        sim.step()
    
    # Reset to a new state
    new_state = {
        'altitude': 15000.0,
        'airspeed': 200.0,
        'heading': 180.0
    }
    sim.reset_aircraft(new_state)
    
    # Check reset state
    assert sim.state['altitude'] == 15000.0
    assert sim.state['airspeed'] == 200.0
    assert sim.state['heading'] == 180.0
    assert sim.time == 0.0


def test_virtual_simulator_logging():
    """Test simulator history logging."""
    sim = VirtualSimulator(enable_logging=True)
    
    # Run several steps
    for _ in range(5):
        sim.step()
    
    # Check history
    history = sim.get_history()
    assert len(history['time']) == 6  # Initial + 5 steps
    assert len(history['altitude']) == 6
    assert len(history['airspeed']) == 6
    assert len(history['heading']) == 6
    
    # Check that time increments correctly
    for i in range(1, 6):
        assert history['time'][i] - history['time'][i-1] == pytest.approx(0.1)


if __name__ == "__main__":
    pytest.main(["-v", __file__])
