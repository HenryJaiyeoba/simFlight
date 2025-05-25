"""
GUI dashboard module for real-time parameter adjustment and visualization.

This module provides a simple graphical user interface for adjusting
controller parameters and visualizing flight data in real-time.
"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import time
import threading
from typing import Dict, Optional


class AutopilotDashboard:
    """
    GUI dashboard for real-time parameter adjustment and visualization.
    
    This class provides a graphical interface for adjusting PID controller
    parameters and visualizing flight data in real-time.
    
    Attributes:
        root: Tkinter root window
        aircraft: The aircraft being controlled
        controllers: Dictionary of autopilot controllers
        figures: Dictionary of matplotlib figures
        canvases: Dictionary of matplotlib canvases
        param_vars: Dictionary of tkinter variables for parameters
        update_interval: Update interval in milliseconds
        running: Whether the dashboard is currently running
    """
    
    def __init__(
        self,
        aircraft,
        controllers: Optional[Dict] = None,
        update_interval: int = 100,
        window_title: str = "easyflight Autopilot Dashboard"
    ):
        """
        Initialize the autopilot dashboard.
        
        Args:
            aircraft: The aircraft being controlled
            controllers: Dictionary of autopilot controllers
            update_interval: Update interval in milliseconds
            window_title: Title for the dashboard window
        """
        self.aircraft = aircraft
        self.controllers = controllers or {}
        self.update_interval = update_interval
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title(window_title)
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Set up the main frame
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create tabs
        self.tab_control = ttk.Notebook(self.main_frame)
        
        # Control tab
        self.control_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.control_tab, text="Controls")
        
        # Visualization tab
        self.viz_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.viz_tab, text="Visualization")
        
        # Navigation tab
        self.nav_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.nav_tab, text="Navigation")
        
        # Add the tabs to the main frame
        self.tab_control.pack(fill=tk.BOTH, expand=True)
        
        # Initialize GUI components
        self.param_vars = {}
        self.state_labels = {}
        self.figures = {}
        self.canvases = {}
        
        # Set up the control tab
        self._setup_control_tab()
        
        # Set up the visualization tab
        self._setup_viz_tab()
        
        # Set up the navigation tab
        self._setup_nav_tab()
        
        # Function that will be called at each update to advance the simulation
        self.simulation_step = None
        
        # Thread for updating the GUI
        self.running = False
        self.update_thread = None
    
    def _setup_control_tab(self):
        """Set up the control tab with parameter adjustment sliders."""
        # Create a frame for each controller
        for i, (name, controller) in enumerate(self.controllers.items()):
            if hasattr(controller, 'controller'):
                # This is an autopilot subsystem with a PID controller
                pid_controller = controller.controller
                frame = self._create_controller_frame(self.control_tab, name, pid_controller)
                frame.grid(row=i//2, column=i%2, padx=10, pady=10, sticky="nsew")
        
        # Make the grid resizable
        rows, cols = (len(self.controllers) + 1) // 2, 2
        for i in range(rows):
            self.control_tab.grid_rowconfigure(i, weight=1)
        for i in range(cols):
            self.control_tab.grid_columnconfigure(i, weight=1)
    
    def _create_controller_frame(self, parent, name, pid_controller):
        """Create a frame with sliders for a PID controller."""
        frame = ttk.LabelFrame(parent, text=name)
        
        # Add sliders for Kp, Ki, Kd
        self._add_param_slider(frame, f"{name}_kp", "Kp", 0, 0.5, pid_controller.kp, 0.001,
                               lambda val: self._update_pid_param(pid_controller, 'kp', val))
        self._add_param_slider(frame, f"{name}_ki", "Ki", 0, 0.1, pid_controller.ki, 0.0001,
                               lambda val: self._update_pid_param(pid_controller, 'ki', val))
        self._add_param_slider(frame, f"{name}_kd", "Kd", 0, 1.0, pid_controller.kd, 0.001,
                               lambda val: self._update_pid_param(pid_controller, 'kd', val))
        
        # Add status indicators
        self._add_state_label(frame, f"{name}_setpoint", "Setpoint:")
        self._add_state_label(frame, f"{name}_error", "Error:")
        self._add_state_label(frame, f"{name}_output", "Output:")
        
        return frame
    
    def _add_param_slider(self, parent, name, label_text, min_val, max_val, default_val, resolution, callback):
        """Add a slider for parameter adjustment."""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Label
        label = ttk.Label(frame, text=label_text, width=10)
        label.pack(side=tk.LEFT)
        
        # Variable
        var = tk.DoubleVar(value=default_val)
        self.param_vars[name] = var
        
        # Slider
        slider = ttk.Scale(frame, from_=min_val, to=max_val, variable=var, 
                          command=lambda val: callback(float(val)))
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Value label
        value_label = ttk.Label(frame, text=f"{default_val:.4f}", width=10)
        value_label.pack(side=tk.RIGHT)
        
        # Update the value label when the slider changes
        def update_label(*args):
            value_label.config(text=f"{var.get():.4f}")
        
        var.trace_add("write", update_label)
        
        return slider
    
    def _add_state_label(self, parent, name, label_text):
        """Add a label for displaying state information."""
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        
        # Label
        label = ttk.Label(frame, text=label_text, width=10)
        label.pack(side=tk.LEFT)
        
        # Value label
        value_label = ttk.Label(frame, text="0.000", width=15)
        value_label.pack(side=tk.LEFT, padx=5)
        
        self.state_labels[name] = value_label
        
        return value_label
    
    def _update_pid_param(self, controller, param, value):
        """Update a PID controller parameter."""
        setattr(controller, param, float(value))
    
    def _setup_viz_tab(self):
        """Set up the visualization tab with plots."""
        # Create a frame for aircraft state visualization
        state_frame = ttk.LabelFrame(self.viz_tab, text="Aircraft State")
        state_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create figures and canvases
        self._create_state_plot(state_frame, "altitude", "Altitude (ft)", "time (s)")
        self._create_state_plot(state_frame, "attitude", "Attitude (deg)", "time (s)", 
                              ["pitch", "roll", "heading"])
        self._create_state_plot(state_frame, "speed", "Speed (knots)", "time (s)")
        self._create_state_plot(state_frame, "controls", "Control Inputs", "time (s)",
                              ["elevator", "aileron", "throttle", "rudder"])
    
    def _create_state_plot(self, parent, name, ylabel, xlabel, lines=None):
        """Create a plot for visualizing aircraft state."""
        # Create figure and subplot
        fig = plt.Figure(figsize=(5, 3), dpi=100)
        ax = fig.add_subplot(111)
        
        # Set labels
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True)
        
        # Add the figure to the parent frame
        canvas = FigureCanvasTkAgg(fig, master=parent)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        
        # Store the figure and canvas
        self.figures[name] = {"fig": fig, "ax": ax, "lines": lines}
        self.canvases[name] = canvas
    
    def _setup_nav_tab(self):
        """Set up the navigation tab with waypoint and autoland controls."""
        # Create frames for waypoint navigation and autoland
        waypoint_frame = ttk.LabelFrame(self.nav_tab, text="Waypoint Navigation")
        waypoint_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        autoland_frame = ttk.LabelFrame(self.nav_tab, text="Autoland")
        autoland_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Set up waypoint navigation controls
        self._setup_waypoint_controls(waypoint_frame)
        
        # Set up autoland controls
        self._setup_autoland_controls(autoland_frame)
    
    def _setup_waypoint_controls(self, parent):
        """Set up controls for waypoint navigation."""
        # Check if we have a waypoint navigation controller
        if "waypoint_navigation" not in self.controllers:
            ttk.Label(parent, text="Waypoint Navigation controller not available").pack(padx=10, pady=10)
            return
        
        nav = self.controllers["waypoint_navigation"]
        
        # Create a frame for the waypoint list
        list_frame = ttk.Frame(parent)
        list_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create a listbox for waypoints
        waypoint_listbox = tk.Listbox(list_frame, height=8)
        waypoint_listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Add a scrollbar
        scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=waypoint_listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        waypoint_listbox.config(yscrollcommand=scrollbar.set)
        
        # Create a frame for waypoint controls
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Add waypoint button
        add_button = ttk.Button(control_frame, text="Add Waypoint", 
                               command=lambda: self._add_waypoint(nav, waypoint_listbox))
        add_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Remove waypoint button
        remove_button = ttk.Button(control_frame, text="Remove Waypoint", 
                                  command=lambda: self._remove_waypoint(nav, waypoint_listbox))
        remove_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Enable/disable navigation button
        self.nav_enabled_var = tk.BooleanVar(value=nav.enabled)
        nav_button = ttk.Checkbutton(control_frame, text="Enable Navigation", 
                                    variable=self.nav_enabled_var,
                                    command=lambda: self._toggle_navigation(nav))
        nav_button.pack(side=tk.LEFT, padx=5, pady=5)
        
        # Status labels
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self._add_state_label(status_frame, "waypoint_current", "Current WP:")
        self._add_state_label(status_frame, "waypoint_distance", "Distance:")
        self._add_state_label(status_frame, "waypoint_bearing", "Bearing:")
        
        # Update the waypoint list
        self._update_waypoint_list(nav, waypoint_listbox)
    
    def _add_waypoint(self, nav_controller, listbox):
        """Add a waypoint to the flight plan."""
        # Create a simple dialog for waypoint input
        dialog = tk.Toplevel(self.root)
        dialog.title("Add Waypoint")
        dialog.geometry("300x200")
        dialog.transient(self.root)
        dialog.grab_set()
        
        # Fields
        ttk.Label(dialog, text="Name:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        name_var = tk.StringVar(value=f"WP_{len(nav_controller.flight_plan.waypoints)+1}")
        ttk.Entry(dialog, textvariable=name_var).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(dialog, text="Latitude:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        lat_var = tk.DoubleVar(value=self.aircraft.state['latitude'])
        ttk.Entry(dialog, textvariable=lat_var).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(dialog, text="Longitude:").grid(row=2, column=0, padx=5, pady=5, sticky="w")
        lon_var = tk.DoubleVar(value=self.aircraft.state['longitude'])
        ttk.Entry(dialog, textvariable=lon_var).grid(row=2, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(dialog, text="Altitude (ft):").grid(row=3, column=0, padx=5, pady=5, sticky="w")
        alt_var = tk.DoubleVar(value=self.aircraft.state['altitude'])
        ttk.Entry(dialog, textvariable=alt_var).grid(row=3, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(dialog, text="Speed (kt):").grid(row=4, column=0, padx=5, pady=5, sticky="w")
        speed_var = tk.DoubleVar(value=self.aircraft.state['airspeed'])
        ttk.Entry(dialog, textvariable=speed_var).grid(row=4, column=1, padx=5, pady=5, sticky="ew")
        
        # Buttons
        def on_ok():
            from ..navigation import Waypoint
            wp = Waypoint(
                latitude=lat_var.get(),
                longitude=lon_var.get(),
                altitude=alt_var.get(),
                speed=speed_var.get(),
                name=name_var.get()
            )
            nav_controller.flight_plan.add_waypoint(wp)
            self._update_waypoint_list(nav_controller, listbox)
            dialog.destroy()
        
        def on_cancel():
            dialog.destroy()
        
        button_frame = ttk.Frame(dialog)
        button_frame.grid(row=5, column=0, columnspan=2, padx=5, pady=10)
        
        ttk.Button(button_frame, text="OK", command=on_ok).pack(side=tk.LEFT, padx=5)
        ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side=tk.LEFT, padx=5)
        
        dialog.wait_window()
    
    def _remove_waypoint(self, nav_controller, listbox):
        """Remove a waypoint from the flight plan."""
        if not listbox.curselection():
            return
        
        index = listbox.curselection()[0]
        nav_controller.flight_plan.remove_waypoint(index)
        self._update_waypoint_list(nav_controller, listbox)
    
    def _update_waypoint_list(self, nav_controller, listbox):
        """Update the waypoint list display."""
        listbox.delete(0, tk.END)
        for i, wp in enumerate(nav_controller.flight_plan.waypoints):
            marker = "→ " if i == nav_controller.flight_plan.current_waypoint_index else "  "
            listbox.insert(tk.END, f"{marker}{wp.name}: {wp.latitude:.4f}°, {wp.longitude:.4f}°, {wp.altitude:.0f}ft, {wp.speed:.0f}kt")
    
    def _toggle_navigation(self, nav_controller):
        """Enable or disable waypoint navigation."""
        if self.nav_enabled_var.get():
            nav_controller.enable()
        else:
            nav_controller.disable()
    
    def _setup_autoland_controls(self, parent):
        """Set up controls for autoland."""
        # Check if we have an autoland controller
        if "autoland" not in self.controllers:
            ttk.Label(parent, text="Autoland controller not available").pack(padx=10, pady=10)
            return
        
        autoland = self.controllers["autoland"]
        
        # Create a frame for runway settings
        runway_frame = ttk.LabelFrame(parent, text="Runway Settings")
        runway_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Runway threshold lat/lon
        ttk.Label(runway_frame, text="Threshold Lat:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        lat_var = tk.DoubleVar(value=autoland.glide_slope.runway_threshold_lat)
        ttk.Entry(runway_frame, textvariable=lat_var).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(runway_frame, text="Threshold Lon:").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        lon_var = tk.DoubleVar(value=autoland.glide_slope.runway_threshold_lon)
        ttk.Entry(runway_frame, textvariable=lon_var).grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        
        # Runway elevation and heading
        ttk.Label(runway_frame, text="Elevation (ft):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        elev_var = tk.DoubleVar(value=autoland.glide_slope.runway_altitude)
        ttk.Entry(runway_frame, textvariable=elev_var).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        ttk.Label(runway_frame, text="Heading (deg):").grid(row=1, column=2, padx=5, pady=5, sticky="w")
        hdg_var = tk.DoubleVar(value=autoland.glide_slope.runway_heading)
        ttk.Entry(runway_frame, textvariable=hdg_var).grid(row=1, column=3, padx=5, pady=5, sticky="ew")
        
        # Set runway button
        def set_runway():
            autoland.set_runway(
                threshold_lat=lat_var.get(),
                threshold_lon=lon_var.get(),
                altitude=elev_var.get(),
                heading=hdg_var.get()
            )
        
        ttk.Button(runway_frame, text="Set Runway", command=set_runway).grid(
            row=2, column=0, columnspan=4, padx=5, pady=5)
        
        # Create a frame for autoland controls
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Approach speed
        ttk.Label(control_frame, text="Approach Speed (kt):").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        speed_var = tk.DoubleVar(value=autoland.target_approach_speed)
        ttk.Entry(control_frame, textvariable=speed_var).grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        # Glide slope
        ttk.Label(control_frame, text="Glide Slope (deg):").grid(row=0, column=2, padx=5, pady=5, sticky="w")
        gs_var = tk.DoubleVar(value=autoland.glide_slope.target)
        ttk.Entry(control_frame, textvariable=gs_var).grid(row=0, column=3, padx=5, pady=5, sticky="ew")
        
        # Flare altitude
        ttk.Label(control_frame, text="Flare Altitude (ft):").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        flare_var = tk.DoubleVar(value=autoland.flare_altitude)
        ttk.Entry(control_frame, textvariable=flare_var).grid(row=1, column=1, padx=5, pady=5, sticky="ew")
        
        # Update settings button
        def update_settings():
            autoland.target_approach_speed = speed_var.get()
            autoland.glide_slope.target = gs_var.get()
            autoland.flare_altitude = flare_var.get()
        
        ttk.Button(control_frame, text="Update Settings", command=update_settings).grid(
            row=1, column=2, columnspan=2, padx=5, pady=5)
        
        # Enable/disable autoland button
        self.autoland_enabled_var = tk.BooleanVar(value=autoland.enabled)
        autoland_button = ttk.Checkbutton(parent, text="Enable Autoland", 
                                         variable=self.autoland_enabled_var,
                                         command=lambda: self._toggle_autoland(autoland))
        autoland_button.pack(fill=tk.X, padx=5, pady=5)
        
        # Status labels
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self._add_state_label(status_frame, "autoland_phase", "Phase:")
        self._add_state_label(status_frame, "autoland_gs_error", "GS Error:")
        self._add_state_label(status_frame, "autoland_loc_error", "LOC Error:")
        self._add_state_label(status_frame, "autoland_radar_alt", "Radar Alt:")
    
    def _toggle_autoland(self, autoland_controller):
        """Enable or disable autoland."""
        if self.autoland_enabled_var.get():
            autoland_controller.enable()
        else:
            autoland_controller.disable()
    
    def start(self):
        """Start the dashboard."""
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop)
        self.update_thread.daemon = True
        self.update_thread.start()
        self.root.mainloop()
    
    def _update_loop(self):
        """Update the dashboard in a background thread."""
        while self.running:
            # Schedule GUI updates on the main thread
            self.root.after(0, self._update_gui)
            
            # Run the simulation step if provided
            if self.simulation_step:
                self.simulation_step()
                
            time.sleep(self.update_interval / 1000)
    
    def _update_gui(self):
        """Update the GUI with current aircraft and controller state."""
        if not self.running:
            return
        
        # Update controller status indicators
        self._update_controller_status()
        
        # Update plots
        self._update_plots()
        
        # Update navigation status
        self._update_navigation_status()
    
    def _update_controller_status(self):
        """Update the status indicators for controllers."""
        for name, controller in self.controllers.items():
            if hasattr(controller, 'controller'):
                # This is an autopilot subsystem with a PID controller
                pid_controller = controller.controller
                
                # Update status labels
                if f"{name}_setpoint" in self.state_labels:
                    self.state_labels[f"{name}_setpoint"].config(
                        text=f"{pid_controller.setpoint:.2f}")
                
                if f"{name}_error" in self.state_labels:
                    if hasattr(controller, 'get_error'):
                        error = controller.get_error()
                    else:
                        error = pid_controller.last_error
                    self.state_labels[f"{name}_error"].config(
                        text=f"{error:.2f}")
                
                if f"{name}_output" in self.state_labels:
                    self.state_labels[f"{name}_output"].config(
                        text=f"{pid_controller.last_output:.3f}")
    
    def _update_plots(self):
        """Update the visualization plots."""
        # Update time axis data (last 60 seconds)
        aircraft_history = self.aircraft.simulator.history
        if not aircraft_history.get('time', []):
            return
        
        # Get the last 60 seconds of data
        times = np.array(aircraft_history.get('time', []))
        if len(times) > 1:
            start_time = max(0, times[-1] - 60)
            start_idx = np.searchsorted(times, start_time)
            plot_times = times[start_idx:] - times[start_idx]
            
            # Update altitude plot
            if 'altitude' in self.figures:
                ax = self.figures['altitude']['ax']
                ax.clear()
                ax.plot(plot_times, aircraft_history['altitude'][start_idx:], 'b-')
                ax.set_xlabel('time (s)')
                ax.set_ylabel('Altitude (ft)')
                ax.grid(True)
                self.canvases['altitude'].draw()
            
            # Update attitude plot
            if 'attitude' in self.figures:
                ax = self.figures['attitude']['ax']
                ax.clear()
                # print(len(plot_times), len(aircraft_history['pitch'][start_idx:]))
                ax.plot(plot_times, aircraft_history['pitch'][start_idx:], 'r-', label='Pitch')
                ax.plot(plot_times, aircraft_history['roll'][start_idx:], 'g-', label='Roll')
                ax.plot(plot_times, aircraft_history['heading'][start_idx:], 'b-', label='Heading')
                ax.set_xlabel('time (s)')
                ax.set_ylabel('Angle (deg)')
                ax.legend()
                ax.grid(True)
                self.canvases['attitude'].draw()
            
            # Update speed plot
            if 'speed' in self.figures:
                ax = self.figures['speed']['ax']
                ax.clear()
                ax.plot(plot_times, aircraft_history['airspeed'][start_idx:], 'b-')
                ax.set_xlabel('time (s)')
                ax.set_ylabel('Airspeed (knots)')
                ax.grid(True)
                self.canvases['speed'].draw()
            
            # Update controls plot
            if 'controls' in self.figures:
                ax = self.figures['controls']['ax']
                ax.clear()
                ax.plot(plot_times, aircraft_history['elevator'][start_idx:], 'r-', label='Elevator')
                ax.plot(plot_times, aircraft_history['aileron'][start_idx:], 'g-', label='Aileron')
                ax.plot(plot_times, aircraft_history['throttle'][start_idx:], 'b-', label='Throttle')
                if 'rudder' in aircraft_history:
                    ax.plot(plot_times, aircraft_history['rudder'][start_idx:], 'y-', label='Rudder')
                ax.set_xlabel('time (s)')
                ax.set_ylabel('Control Input')
                ax.legend()
                ax.grid(True)
                self.canvases['controls'].draw()
    
    def _update_navigation_status(self):
        """Update the navigation status indicators."""
        # Update waypoint navigation status
        if "waypoint_navigation" in self.controllers:
            nav = self.controllers["waypoint_navigation"]
            waypoint = nav.flight_plan.get_current_waypoint()
            
            if waypoint and "waypoint_current" in self.state_labels:
                self.state_labels["waypoint_current"].config(text=waypoint.name)
            
            if "waypoint_distance" in self.state_labels:
                self.state_labels["waypoint_distance"].config(
                    text=f"{nav.distance_to_waypoint/1000:.2f} km")
            
            if "waypoint_bearing" in self.state_labels:
                self.state_labels["waypoint_bearing"].config(
                    text=f"{nav.bearing_to_waypoint:.1f}°")
        
        # Update autoland status
        if "autoland" in self.controllers:
            autoland = self.controllers["autoland"]
            
            if "autoland_phase" in self.state_labels:
                self.state_labels["autoland_phase"].config(text=autoland.phase)
            
            if "autoland_gs_error" in self.state_labels:
                self.state_labels["autoland_gs_error"].config(
                    text=f"{autoland.glide_slope.get_error():.2f}°")
            
            if "autoland_loc_error" in self.state_labels:
                self.state_labels["autoland_loc_error"].config(
                    text=f"{autoland.localizer.get_error():.1f}m")
            
            if "autoland_radar_alt" in self.state_labels:
                radar_alt = autoland.aircraft.state['altitude'] - autoland.glide_slope.runway_altitude
                self.state_labels["autoland_radar_alt"].config(
                    text=f"{radar_alt:.1f}ft")
    
    def on_close(self):
        """Handle window close event."""
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=1.0)
        self.root.destroy()
