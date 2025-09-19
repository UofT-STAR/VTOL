"""
Thrust Vector Controller for VTOL rocket
Manages thrust vectoring for vertical landing and takeoff
"""

import math
import numpy as np
from typing import Tuple, NamedTuple, List
from .attitude_controller import AttitudeController, Attitude
from .pid_controller import PIDController


class ThrustVector(NamedTuple):
    """Thrust vector representation"""
    magnitude: float  # Total thrust magnitude (0-1)
    angle_x: float    # Thrust vector angle in X direction (radians)
    angle_y: float    # Thrust vector angle in Y direction (radians)


class Position(NamedTuple):
    """3D position representation"""
    x: float
    y: float
    z: float


class Velocity(NamedTuple):
    """3D velocity representation"""
    vx: float
    vy: float
    vz: float


class ThrustVectorController:
    """
    Main thrust vector control system for VTOL rocket
    Combines attitude control with position/velocity control
    """
    
    def __init__(self,
                 max_thrust_angle: float = math.radians(15),  # Maximum thrust vector angle
                 altitude_gains: Tuple[float, float, float] = (1.0, 0.02, 0.8),
                 position_gains: Tuple[float, float, float] = (0.5, 0.01, 0.3)):
        """
        Initialize thrust vector controller
        
        Args:
            max_thrust_angle: Maximum allowed thrust vector angle (radians)
            altitude_gains: (kp, ki, kd) gains for altitude control
            position_gains: (kp, ki, kd) gains for horizontal position control
        """
        self.max_thrust_angle = max_thrust_angle
        
        # Sub-controllers
        self.attitude_controller = AttitudeController()
        self.altitude_controller = PIDController(*altitude_gains, output_min=0.0, output_max=1.0)
        self.position_x_controller = PIDController(*position_gains, output_min=-1.0, output_max=1.0)
        self.position_y_controller = PIDController(*position_gains, output_min=-1.0, output_max=1.0)
        
        # Landing mode parameters
        self.landing_mode = False
        self.landing_target_altitude = 0.0
        self.landing_descent_rate = -2.0  # m/s
        
    def update(self,
               target_position: Position,
               current_position: Position,
               current_velocity: Velocity,
               current_attitude: Attitude,
               dt: float = None) -> Tuple[ThrustVector, Attitude]:
        """
        Update thrust vector control
        
        Args:
            target_position: Desired position
            current_position: Current measured position
            current_velocity: Current measured velocity
            current_attitude: Current measured attitude
            dt: Time step (optional)
            
        Returns:
            Tuple of (thrust_vector, target_attitude)
        """
        # Altitude control
        if self.landing_mode:
            # Landing sequence - controlled descent
            thrust_magnitude = self._landing_thrust_control(
                current_position.z, current_velocity.vz, dt
            )
        else:
            # Normal altitude hold
            thrust_magnitude = self.altitude_controller.update(
                target_position.z, current_position.z, dt
            )
        
        # Horizontal position control
        x_control = self.position_x_controller.update(
            target_position.x, current_position.x, dt
        )
        y_control = self.position_y_controller.update(
            target_position.y, current_position.y, dt
        )
        
        # Convert horizontal control to thrust vector angles
        thrust_angle_x = np.clip(x_control * self.max_thrust_angle, 
                                -self.max_thrust_angle, self.max_thrust_angle)
        thrust_angle_y = np.clip(y_control * self.max_thrust_angle,
                                -self.max_thrust_angle, self.max_thrust_angle)
        
        # Calculate desired attitude for stability
        target_attitude = Attitude(
            roll=-thrust_angle_y,   # Roll opposes Y thrust vector
            pitch=thrust_angle_x,   # Pitch matches X thrust vector
            yaw=0.0                 # Maintain heading
        )
        
        # Apply attitude control
        attitude_output = self.attitude_controller.update(
            target_attitude, current_attitude, dt
        )
        
        # Create thrust vector
        thrust_vector = ThrustVector(
            magnitude=thrust_magnitude,
            angle_x=thrust_angle_x,
            angle_y=thrust_angle_y
        )
        
        return thrust_vector, attitude_output
    
    def _landing_thrust_control(self, current_altitude: float, current_velocity_z: float, dt: float) -> float:
        """
        Special thrust control for landing sequence
        
        Args:
            current_altitude: Current altitude above ground
            current_velocity_z: Current vertical velocity
            dt: Time step
            
        Returns:
            Thrust magnitude (0-1)
        """
        # Target velocity based on altitude
        if current_altitude > 10.0:
            target_velocity = self.landing_descent_rate
        elif current_altitude > 2.0:
            # Slow down as we approach ground
            target_velocity = self.landing_descent_rate * 0.5
        else:
            # Very slow final approach
            target_velocity = self.landing_descent_rate * 0.2
            
        # Velocity control
        velocity_error = target_velocity - current_velocity_z
        
        # Base thrust to counteract gravity (approximately 1.0)
        # Add velocity correction
        thrust = 1.0 + velocity_error * 0.5
        
        # Clamp thrust
        return np.clip(thrust, 0.0, 1.0)
    
    def initiate_landing(self, target_altitude: float = 0.0):
        """
        Initiate landing sequence
        
        Args:
            target_altitude: Target landing altitude
        """
        self.landing_mode = True
        self.landing_target_altitude = target_altitude
        
    def abort_landing(self):
        """Abort landing sequence and return to normal control"""
        self.landing_mode = False
        
    def reset(self):
        """Reset all controllers"""
        self.attitude_controller.reset()
        self.altitude_controller.reset()
        self.position_x_controller.reset()
        self.position_y_controller.reset()
        self.landing_mode = False
        
    def set_max_thrust_angle(self, angle: float):
        """Set maximum thrust vector angle"""
        self.max_thrust_angle = angle
        
    def get_status(self) -> dict:
        """Get controller status"""
        return {
            'landing_mode': self.landing_mode,
            'max_thrust_angle_deg': math.degrees(self.max_thrust_angle),
            'landing_target_altitude': self.landing_target_altitude
        }