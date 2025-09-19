"""
Attitude Controller for VTOL rocket
Manages roll, pitch, and yaw control
"""

import math
import numpy as np
from typing import Tuple, NamedTuple
from .pid_controller import PIDController


class Attitude(NamedTuple):
    """Attitude representation in Euler angles (radians)"""
    roll: float
    pitch: float
    yaw: float


class AttitudeController:
    """
    Attitude controller for VTOL rocket
    Manages orientation control using PID controllers for each axis
    """
    
    def __init__(self, 
                 roll_gains: Tuple[float, float, float] = (2.0, 0.1, 0.5),
                 pitch_gains: Tuple[float, float, float] = (2.0, 0.1, 0.5),
                 yaw_gains: Tuple[float, float, float] = (1.5, 0.05, 0.3)):
        """
        Initialize attitude controller
        
        Args:
            roll_gains: (kp, ki, kd) gains for roll control
            pitch_gains: (kp, ki, kd) gains for pitch control  
            yaw_gains: (kp, ki, kd) gains for yaw control
        """
        self.roll_controller = PIDController(*roll_gains, output_min=-1.0, output_max=1.0)
        self.pitch_controller = PIDController(*pitch_gains, output_min=-1.0, output_max=1.0)
        self.yaw_controller = PIDController(*yaw_gains, output_min=-1.0, output_max=1.0)
        
    def update(self, target_attitude: Attitude, current_attitude: Attitude, dt: float = None) -> Attitude:
        """
        Update attitude control
        
        Args:
            target_attitude: Desired attitude
            current_attitude: Current measured attitude
            dt: Time step (optional)
            
        Returns:
            Control outputs for roll, pitch, yaw
        """
        # Calculate control outputs for each axis
        roll_output = self.roll_controller.update(target_attitude.roll, current_attitude.roll, dt)
        pitch_output = self.pitch_controller.update(target_attitude.pitch, current_attitude.pitch, dt)
        yaw_output = self.yaw_controller.update(target_attitude.yaw, current_attitude.yaw, dt)
        
        return Attitude(roll_output, pitch_output, yaw_output)
    
    def reset(self):
        """Reset all attitude controllers"""
        self.roll_controller.reset()
        self.pitch_controller.reset()
        self.yaw_controller.reset()
        
    def set_gains(self, 
                  roll_gains: Tuple[float, float, float] = None,
                  pitch_gains: Tuple[float, float, float] = None,
                  yaw_gains: Tuple[float, float, float] = None):
        """Update controller gains"""
        if roll_gains:
            self.roll_controller.set_gains(*roll_gains)
        if pitch_gains:
            self.pitch_controller.set_gains(*pitch_gains)
        if yaw_gains:
            self.yaw_controller.set_gains(*yaw_gains)