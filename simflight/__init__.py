"""
simFlight: Aircraft Autopilot Control Systems Library

A beginner-friendly, educational Python library for teaching the principles
of aircraft autopilot control systems with a focus on PID controllers.
"""

__version__ = '0.1.0'
__author__ = 'simFlight Team'

# Import key modules to make them directly accessible
from .controllers import PIDController
from .aircraft import Aircraft
from .autopilot import AltitudeHold, HeadingHold, SpeedHold, Autoland
from .simulators import VirtualSimulator, XPlaneInterface
from .navigation import Waypoint, FlightPlan, WaypointNavigation
from .visualization import plot_simulation_results, plot_pid_performance, AutopilotDashboard
