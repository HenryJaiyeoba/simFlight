"""
Tests for the waypoint navigation functionality.
"""

import unittest
import numpy as np
from math import radians, degrees

from easyflight.simulators import VirtualSimulator
from easyflight.aircraft import Aircraft
from easyflight.autopilot import AltitudeHold, HeadingHold, SpeedHold
from easyflight.navigation import Waypoint, FlightPlan, WaypointNavigation


class TestWaypointNavigation(unittest.TestCase):
    """Tests for the waypoint navigation functionality."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create a simulator with a known initial state
        initial_state = {
            'altitude': 5000.0,
            'airspeed': 120.0,
            'heading': 0.0,
            'pitch': 0.0,
            'roll': 0.0,
            'latitude': 37.0,
            'longitude': -122.0
        }
        self.simulator = VirtualSimulator(initial_state=initial_state)
        self.aircraft = Aircraft(self.simulator)
        
        # Create the navigation system
        self.nav = WaypointNavigation(self.aircraft)
    
    def test_waypoint_creation(self):
        """Test creation of waypoints."""
        wp = Waypoint(37.1, -122.1, 6000.0, 150.0, "TEST_WP")
        self.assertEqual(wp.latitude, 37.1)
        self.assertEqual(wp.longitude, -122.1)
        self.assertEqual(wp.altitude, 6000.0)
        self.assertEqual(wp.speed, 150.0)
        self.assertEqual(wp.name, "TEST_WP")
    
    def test_flight_plan_operations(self):
        """Test flight plan operations."""
        # Create a flight plan
        flight_plan = FlightPlan(name="Test Plan")
        
        # Add waypoints
        wp1 = Waypoint(37.1, -122.1, 6000.0, 150.0, "WP1")
        wp2 = Waypoint(37.2, -122.2, 7000.0, 140.0, "WP2")
        flight_plan.add_waypoint(wp1)
        flight_plan.add_waypoint(wp2)
        
        # Check flight plan properties
        self.assertEqual(len(flight_plan), 2)
        self.assertEqual(flight_plan.name, "Test Plan")
        self.assertEqual(flight_plan.current_waypoint_index, 0)
        
        # Test waypoint navigation
        self.assertEqual(flight_plan.get_current_waypoint(), wp1)
        self.assertEqual(flight_plan.next_waypoint(), wp2)
        self.assertEqual(flight_plan.current_waypoint_index, 1)
        self.assertEqual(flight_plan.previous_waypoint(), wp1)
        self.assertEqual(flight_plan.current_waypoint_index, 0)
        
        # Test removing a waypoint
        flight_plan.remove_waypoint(0)
        self.assertEqual(len(flight_plan), 1)
        self.assertEqual(flight_plan.get_current_waypoint(), wp2)
    
    def test_haversine_distance(self):
        """Test the haversine distance calculation."""
        # San Francisco to Los Angeles: ~559 km
        sf_lat, sf_lon = 37.7749, -122.4194
        la_lat, la_lon = 34.0522, -118.2437
        
        distance = self.nav._haversine_distance(sf_lat, sf_lon, la_lat, la_lon)
        
        # Check that the distance is approximately correct (within 1%)
        self.assertAlmostEqual(distance/1000, 559.0, delta=6.0)
    
    def test_bearing_calculation(self):
        """Test the bearing calculation."""
        # North
        bearing = self.nav._calculate_bearing(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(bearing, 0.0, delta=1.0)
        
        # East
        bearing = self.nav._calculate_bearing(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(bearing, 90.0, delta=1.0)
        
        # South
        bearing = self.nav._calculate_bearing(0.0, 0.0, -1.0, 0.0)
        self.assertAlmostEqual(bearing, 180.0, delta=1.0)
        
        # West
        bearing = self.nav._calculate_bearing(0.0, 0.0, 0.0, -1.0)
        self.assertAlmostEqual(bearing, 270.0, delta=1.0)
    
    def test_waypoint_reached(self):
        """Test the waypoint reached detection."""
        # Create a waypoint very close to the aircraft
        wp = Waypoint(37.0, -122.0, 5000.0, 120.0, "CLOSE_WP")
        
        # Create a flight plan with this waypoint
        flight_plan = FlightPlan()
        flight_plan.add_waypoint(wp)
        
        # Set up navigation
        self.nav.flight_plan = flight_plan
        self.nav.enable()
        
        # The waypoint should be considered reached immediately
        result = self.nav.update()
        self.assertTrue(result)
    
    def test_controller_integration(self):
        """Test integration with autopilot controllers."""
        # Create controllers
        altitude_hold = AltitudeHold()
        heading_hold = HeadingHold()
        speed_hold = SpeedHold()
        
        # Create navigation with these controllers
        nav = WaypointNavigation(
            self.aircraft,
            heading_controller=heading_hold,
            altitude_controller=altitude_hold,
            speed_controller=speed_hold
        )
        
        # Create a waypoint
        wp = Waypoint(37.1, -122.1, 6000.0, 150.0, "TARGET_WP")
        
        # Create a flight plan with this waypoint
        flight_plan = FlightPlan()
        flight_plan.add_waypoint(wp)
        
        # Set up navigation
        nav.flight_plan = flight_plan
        nav.enable()
        
        # Check that the controllers are set to the correct targets
        self.assertEqual(altitude_hold.target, 6000.0)
        self.assertAlmostEqual(speed_hold.target, 150.0)
        
        # The heading should be set to point toward the waypoint
        # We don't check the exact value as it depends on the bearings


if __name__ == '__main__':
    unittest.main()
