"""
Visualization module for aircraft control systems.

This module provides functions for visualizing aircraft state and controller
performance using matplotlib and plotly.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Any, Optional, Tuple, Union
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def plot_simulation_results(
    time: List[float],
    state_history: Dict[str, List[float]],
    setpoints: Optional[Dict[str, float]] = None,
    control_history: Optional[Dict[str, List[float]]] = None,
    variables: Optional[List[str]] = None,
    fig_size: Tuple[int, int] = (12, 8)
) -> plt.Figure:
    """
    Plot the results of a simulation using matplotlib.
    
    Args:
        time: List of time points
        state_history: Dictionary of state variable histories
        setpoints: Dictionary of setpoint values
        control_history: Dictionary of control input histories
        variables: List of state variables to plot
        fig_size: Figure size (width, height) in inches
        
    Returns:
        matplotlib Figure object
    """
    if variables is None:
        variables = ['altitude', 'airspeed', 'heading', 'pitch', 'roll']
    
    n_vars = len(variables)
    n_rows = n_vars + (1 if control_history else 0)
    
    fig, axes = plt.subplots(n_rows, 1, figsize=fig_size, sharex=True)
    
    if n_rows == 1:
        axes = [axes]
    
    # Plot state variables
    for i, var in enumerate(variables):
        if var in state_history:
            axes[i].plot(time, state_history[var], 'b-', label=f'Actual {var}')
            
            if setpoints and var in setpoints:
                axes[i].axhline(y=setpoints[var], color='r', linestyle='--', label=f'Target {var}')
                
            axes[i].set_ylabel(var_to_label(var))
            axes[i].legend()
            axes[i].grid(True)
    
    # Plot control inputs
    if control_history:
        control_ax = axes[-1]
        control_vars = list(control_history.keys())
        
        for var in control_vars:
            control_ax.plot(time, control_history[var], label=var)
            
        control_ax.set_ylabel('Control Inputs')
        control_ax.set_xlabel('Time (s)')
        control_ax.legend()
        control_ax.grid(True)
    else:
        axes[-1].set_xlabel('Time (s)')
    
    plt.tight_layout()
    return fig


def plot_pid_performance(
    controller_history: Dict[str, List[float]],
    fig_size: Tuple[int, int] = (12, 10)
) -> plt.Figure:
    """
    Plot the performance of a PID controller using matplotlib.
    
    Args:
        controller_history: Dictionary of controller history
        fig_size: Figure size (width, height) in inches
        
    Returns:
        matplotlib Figure object
    """
    time = controller_history['time']
    time = [t - time[0] for t in time]  # Convert to relative time
    
    fig, axes = plt.subplots(4, 1, figsize=fig_size, sharex=True)
    
    # Plot setpoint vs process variable
    axes[0].plot(time, controller_history['process_variable'], 'b-', label='Process Variable')
    axes[0].plot(time, controller_history['setpoint'], 'r--', label='Setpoint')
    axes[0].set_ylabel('Value')
    axes[0].legend()
    axes[0].grid(True)
    
    # Plot error
    axes[1].plot(time, controller_history['error'], 'g-', label='Error')
    axes[1].set_ylabel('Error')
    axes[1].legend()
    axes[1].grid(True)
    
    # Plot PID terms
    axes[2].plot(time, controller_history['p_term'], 'r-', label='P Term')
    axes[2].plot(time, controller_history['i_term'], 'g-', label='I Term')
    axes[2].plot(time, controller_history['d_term'], 'b-', label='D Term')
    axes[2].set_ylabel('PID Terms')
    axes[2].legend()
    axes[2].grid(True)
    
    # Plot output
    axes[3].plot(time, controller_history['output'], 'k-', label='Output')
    axes[3].set_ylabel('Output')
    axes[3].set_xlabel('Time (s)')
    axes[3].legend()
    axes[3].grid(True)
    
    plt.tight_layout()
    return fig


def interactive_plot(
    time: List[float],
    state_history: Dict[str, List[float]],
    setpoints: Optional[Dict[str, float]] = None,
    control_history: Optional[Dict[str, List[float]]] = None,
    variables: Optional[List[str]] = None,
    controller_history: Optional[Dict[str, Dict[str, List[float]]]] = None,
    title: str = "Flight Simulation Results"
) -> go.Figure:
    """
    Create an interactive plot using plotly.
    
    Args:
        time: List of time points
        state_history: Dictionary of state variable histories
        setpoints: Dictionary of setpoint values
        control_history: Dictionary of control input histories
        variables: List of state variables to plot
        controller_history: Dictionary of controller histories by controller name
        title: Plot title
        
    Returns:
        plotly Figure object
    """
    if variables is None:
        variables = ['altitude', 'airspeed', 'heading', 'pitch', 'roll']
    
    n_vars = len(variables)
    n_rows = n_vars + (1 if control_history else 0) + (1 if controller_history else 0)
    
    fig = make_subplots(
        rows=n_rows,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.02,
        subplot_titles=[var_to_label(var) for var in variables] + 
                       (['Control Inputs'] if control_history else []) +
                       (['PID Controller Performance'] if controller_history else [])
    )
    
    # Plot state variables
    for i, var in enumerate(variables):
        if var in state_history:
            fig.add_trace(
                go.Scatter(
                    x=time,
                    y=state_history[var],
                    mode='lines',
                    name=f'Actual {var}'
                ),
                row=i+1,
                col=1
            )
            
            if setpoints and var in setpoints:
                fig.add_trace(
                    go.Scatter(
                        x=[time[0], time[-1]],
                        y=[setpoints[var], setpoints[var]],
                        mode='lines',
                        name=f'Target {var}',
                        line=dict(dash='dash', color='red')
                    ),
                    row=i+1,
                    col=1
                )
    
    # Plot control inputs
    if control_history:
        row = n_vars + 1
        for var, values in control_history.items():
            fig.add_trace(
                go.Scatter(
                    x=time,
                    y=values,
                    mode='lines',
                    name=var
                ),
                row=row,
                col=1
            )
    
    # Plot controller performance
    if controller_history:
        row = n_vars + (2 if control_history else 1)
        for controller_name, history in controller_history.items():
            # Only plot the first controller for simplicity
            controller_time = history['time']
            controller_time = [t - controller_time[0] for t in controller_time]  # Convert to relative time
            
            # Plot error
            fig.add_trace(
                go.Scatter(
                    x=controller_time,
                    y=history['error'],
                    mode='lines',
                    name=f'{controller_name} Error'
                ),
                row=row,
                col=1
            )
            
            # Plot PID terms
            fig.add_trace(
                go.Scatter(
                    x=controller_time,
                    y=history['p_term'],
                    mode='lines',
                    name=f'{controller_name} P Term'
                ),
                row=row,
                col=1
            )
            
            fig.add_trace(
                go.Scatter(
                    x=controller_time,
                    y=history['i_term'],
                    mode='lines',
                    name=f'{controller_name} I Term'
                ),
                row=row,
                col=1
            )
            
            fig.add_trace(
                go.Scatter(
                    x=controller_time,
                    y=history['d_term'],
                    mode='lines',
                    name=f'{controller_name} D Term'
                ),
                row=row,
                col=1
            )
            
            # Only plot the first controller
            break
    
    # Update layout
    fig.update_layout(
        title=title,
        height=250 * n_rows,
        width=1000,
        legend=dict(
            orientation="h",
            yanchor="bottom",
            y=1.02,
            xanchor="right",
            x=1
        ),
        hovermode="x unified"
    )
    
    fig.update_xaxes(title_text="Time (s)", row=n_rows, col=1)
    
    return fig


def var_to_label(var: str) -> str:
    """
    Convert variable name to a readable label with units.
    
    Args:
        var: Variable name
        
    Returns:
        str: Readable label with units
    """
    labels = {
        'altitude': 'Altitude (ft)',
        'vertical_speed': 'Vertical Speed (ft/min)',
        'pitch': 'Pitch (deg)',
        'roll': 'Roll (deg)',
        'heading': 'Heading (deg)',
        'airspeed': 'Airspeed (knots)',
        'throttle': 'Throttle (0-1)',
        'elevator': 'Elevator (-1 to 1)',
        'aileron': 'Aileron (-1 to 1)',
        'rudder': 'Rudder (-1 to 1)'
    }
    
    return labels.get(var, var)
