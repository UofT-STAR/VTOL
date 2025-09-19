"""
PID Controller implementation for VTOL control systems
"""

import time
from typing import Optional


class PIDController:
    """
    Proportional-Integral-Derivative (PID) controller
    Used for controlling various aspects of the VTOL rocket
    """
    
    def __init__(self, kp: float, ki: float, kd: float, 
                 output_min: Optional[float] = None, 
                 output_max: Optional[float] = None):
        """
        Initialize PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain  
            kd: Derivative gain
            output_min: Minimum output value (optional)
            output_max: Maximum output value (optional)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        # Internal state
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        
    def update(self, setpoint: float, measured_value: float, dt: Optional[float] = None) -> float:
        """
        Update PID controller with new measurement
        
        Args:
            setpoint: Desired value
            measured_value: Current measured value
            dt: Time step (optional, will use system time if not provided)
            
        Returns:
            Control output
        """
        current_time = time.time()
        
        if self.previous_time is None:
            self.previous_time = current_time
            
        if dt is None:
            dt = current_time - self.previous_time
            
        # Avoid division by zero
        if dt <= 0.0:
            dt = 1e-6
            
        # Calculate error
        error = setpoint - measured_value
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term
        self.integral += error * dt
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Apply output limits
        if self.output_min is not None and output < self.output_min:
            output = self.output_min
        if self.output_max is not None and output > self.output_max:
            output = self.output_max
            
        # Update state
        self.previous_error = error
        self.previous_time = current_time
        
        return output
    
    def reset(self):
        """Reset the PID controller state"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None
        
    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd