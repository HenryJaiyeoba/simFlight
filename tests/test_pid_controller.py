"""
Tests for the PIDController class.
"""

import pytest
import numpy as np
from simflight.controllers import PIDController


def test_pid_controller_initialization():
    """Test PIDController initialization with default values."""
    pid = PIDController()
    assert pid.kp == 0.0
    assert pid.ki == 0.0
    assert pid.kd == 0.0
    assert pid.setpoint == 0.0
    assert pid.dt == 0.1
    assert pid.output_limits is None
    assert pid.anti_windup is True
    assert pid.anti_windup_limits is None
    assert pid.enable_logging is True


def test_pid_controller_initialization_with_params():
    """Test PIDController initialization with custom values."""
    pid = PIDController(
        kp=1.0,
        ki=0.1,
        kd=0.5,
        setpoint=100.0,
        dt=0.05,
        output_limits=(-1.0, 1.0),
        anti_windup=False,
        enable_logging=False
    )
    assert pid.kp == 1.0
    assert pid.ki == 0.1
    assert pid.kd == 0.5
    assert pid.setpoint == 100.0
    assert pid.dt == 0.05
    assert pid.output_limits == (-1.0, 1.0)
    assert pid.anti_windup is False
    assert pid.enable_logging is False


def test_pid_controller_compute_proportional():
    """Test proportional-only control."""
    pid = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=100.0, enable_logging=False)
    
    # Process variable = 80, error = 20, P term = 20
    output = pid.compute(80.0)
    assert output == pytest.approx(20.0)
    
    # Process variable = 110, error = -10, P term = -10
    output = pid.compute(110.0)
    assert output == pytest.approx(-10.0)


def test_pid_controller_compute_proportional_integral():
    """Test proportional-integral control."""
    pid = PIDController(kp=1.0, ki=0.1, kd=0.0, setpoint=100.0, dt=1.0, enable_logging=False)
    
    # First call: PV = 80, error = 20, P = 20, I = 2
    output = pid.compute(80.0)
    assert output == pytest.approx(22.0)
    
    # Second call: PV = 90, error = 10, P = 10, I = 2 + 1 = 3
    output = pid.compute(90.0)
    assert output == pytest.approx(13.0)
    
    # Third call: PV = 110, error = -10, P = -10, I = 3 - 1 = 2
    output = pid.compute(110.0)
    assert output == pytest.approx(-8.0)


def test_pid_controller_compute_full_pid():
    """Test full PID control."""
    pid = PIDController(kp=1.0, ki=0.1, kd=0.5, setpoint=100.0, dt=1.0, enable_logging=False)
    
    # First call: PV = 80, error = 20, P = 20, I = 2, D = 0 (no previous error)
    output = pid.compute(80.0)
    assert output == pytest.approx(22.0)
    
    # Second call: PV = 90, error = 10, P = 10, I = 3, D = 0.5 * (10 - 20) / 1.0 = -5
    output = pid.compute(90.0)
    assert output == pytest.approx(8.0)
    
    # Third call: PV = 95, error = 5, P = 5, I = 3.5, D = 0.5 * (5 - 10) / 1.0 = -2.5
    output = pid.compute(95.0)
    assert output == pytest.approx(6.0)


def test_pid_controller_output_limits():
    """Test output limits."""
    pid = PIDController(
        kp=10.0, ki=0.0, kd=0.0, setpoint=100.0, 
        output_limits=(-5.0, 5.0), enable_logging=False
    )
    
    # Process variable = 80, error = 20, P term = 200, but limited to 5
    output = pid.compute(80.0)
    assert output == pytest.approx(5.0)
    
    # Process variable = 110, error = -10, P term = -100, but limited to -5
    output = pid.compute(110.0)
    assert output == pytest.approx(-5.0)


def test_pid_controller_anti_windup():
    """Test anti-windup protection."""
    pid = PIDController(
        kp=0.0, ki=1.0, kd=0.0, setpoint=100.0, dt=1.0,
        output_limits=(-5.0, 5.0), anti_windup=True,
        anti_windup_limits=(-5.0, 5.0), enable_logging=False
    )
    
    # First call: PV = 80, error = 20, I = 20, but limited to 5
    output = pid.compute(80.0)
    assert output == pytest.approx(5.0)
    assert pid.integral == pytest.approx(5.0)  # Integral is clamped
    
    # Second call: PV = 90, error = 10, I would be 5 + 10 = 15, but limited to 5
    output = pid.compute(90.0)
    assert output == pytest.approx(5.0)
    assert pid.integral == pytest.approx(5.0)  # Integral remains clamped


def test_pid_controller_reset():
    """Test controller reset."""
    pid = PIDController(
        kp=1.0, ki=0.1, kd=0.5, setpoint=100.0, dt=1.0, enable_logging=False
    )
    
    # Run some computations
    pid.compute(80.0)
    pid.compute(90.0)
    
    # Now reset
    pid.reset()
    assert pid.integral == 0.0
    assert pid.last_error == 0.0


def test_pid_controller_set_gains():
    """Test updating controller gains."""
    pid = PIDController(kp=1.0, ki=0.1, kd=0.5, enable_logging=False)
    
    # Update gains
    pid.set_gains(kp=2.0, ki=0.2, kd=1.0)
    assert pid.kp == 2.0
    assert pid.ki == 0.2
    assert pid.kd == 1.0
    
    # Update only kp
    pid.set_gains(kp=3.0)
    assert pid.kp == 3.0
    assert pid.ki == 0.2
    assert pid.kd == 1.0


def test_pid_controller_set_setpoint():
    """Test updating controller setpoint."""
    pid = PIDController(setpoint=100.0, enable_logging=False)
    
    # Update setpoint
    pid.set_setpoint(200.0)
    assert pid.setpoint == 200.0


def test_pid_controller_logging():
    """Test controller history logging."""
    # Clear any existing histories that might interfere with the test
    pid = PIDController(
        kp=1.0, ki=0.1, kd=0.5, setpoint=100.0, dt=1.0, enable_logging=True
    )
    
    # Reset the history explicitly before the test
    for key in pid.history:
        pid.history[key] = []
    
    # Run some computations
    pid.compute(80.0)
    pid.compute(90.0)
    
    # Check history
    history = pid.get_history()
    assert len(history['time']) == 2
    assert len(history['setpoint']) == 2
    assert len(history['process_variable']) == 2
    assert len(history['error']) == 2
    assert len(history['p_term']) == 2
    assert len(history['i_term']) == 2
    assert len(history['d_term']) == 2
    assert len(history['output']) == 2
    
    # Check values
    assert history['process_variable'][0] == 80.0
    assert history['process_variable'][1] == 90.0
    assert history['error'][0] == 20.0
    assert history['error'][1] == 10.0


if __name__ == "__main__":
    pytest.main(["-v", __file__])
