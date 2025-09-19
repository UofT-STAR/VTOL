"""
Rocket dynamics simulation
Models the physics of the VTOL rocket
"""

import math
import numpy as np
from typing import NamedTuple
from ..control.attitude_controller import Attitude
from ..control.thrust_vector_controller import Position, Velocity, ThrustVector


class RocketState(NamedTuple):
    """Complete state of the rocket"""
    position: Position
    velocity: Velocity
    attitude: Attitude
    angular_velocity: Attitude
    mass: float
    fuel_remaining: float


class RocketDynamics:
    """
    Physics simulation for VTOL rocket
    Includes mass, inertia, thrust, gravity, and atmospheric effects
    """
    
    def __init__(self,
                 dry_mass: float = 500.0,        # kg (empty rocket mass)
                 fuel_mass: float = 200.0,       # kg (fuel mass)
                 max_thrust: float = 8000.0,     # N (maximum thrust)
                 length: float = 10.0,           # m (rocket length)
                 diameter: float = 1.0,          # m (rocket diameter)
                 drag_coefficient: float = 0.5): # drag coefficient
        """
        Initialize rocket dynamics
        
        Args:
            dry_mass: Empty rocket mass (kg)
            fuel_mass: Fuel mass (kg)
            max_thrust: Maximum thrust force (N)
            length: Rocket length (m)
            diameter: Rocket diameter (m)
            drag_coefficient: Aerodynamic drag coefficient
        """
        self.dry_mass = dry_mass
        self.initial_fuel_mass = fuel_mass
        self.max_thrust = max_thrust
        self.length = length
        self.diameter = diameter
        self.drag_coefficient = drag_coefficient
        
        # Physical constants
        self.g = 9.81  # m/s^2 (gravity)
        self.air_density = 1.225  # kg/m^3 (sea level air density)
        self.fuel_consumption_rate = 0.3  # kg/s per unit thrust
        
        # Moments of inertia (simplified as cylinder)
        self.cross_sectional_area = math.pi * (diameter / 2) ** 2
        
        # Initialize state
        self.reset()
        
    def reset(self, 
              initial_position: Position = Position(0, 0, 0),
              initial_velocity: Velocity = Velocity(0, 0, 0),
              initial_attitude: Attitude = Attitude(0, 0, 0)):
        """Reset rocket to initial state"""
        self.state = RocketState(
            position=initial_position,
            velocity=initial_velocity,
            attitude=initial_attitude,
            angular_velocity=Attitude(0, 0, 0),
            mass=self.dry_mass + self.initial_fuel_mass,
            fuel_remaining=self.initial_fuel_mass
        )
        
    def update(self, thrust_vector: ThrustVector, attitude_control: Attitude, dt: float) -> RocketState:
        """
        Update rocket dynamics
        
        Args:
            thrust_vector: Thrust vector command
            attitude_control: Attitude control torques
            dt: Time step
            
        Returns:
            Updated rocket state
        """
        # Current state
        pos = self.state.position
        vel = self.state.velocity
        att = self.state.attitude
        ang_vel = self.state.angular_velocity
        mass = self.state.mass
        fuel = self.state.fuel_remaining
        
        # Calculate thrust force
        thrust_magnitude = thrust_vector.magnitude * self.max_thrust
        
        # Thrust vector in body frame
        thrust_body = np.array([
            thrust_magnitude * math.sin(thrust_vector.angle_x),
            thrust_magnitude * math.sin(thrust_vector.angle_y),
            thrust_magnitude * math.cos(thrust_vector.angle_x) * math.cos(thrust_vector.angle_y)
        ])
        
        # Transform thrust to world frame using attitude
        thrust_world = self._body_to_world(thrust_body, att)
        
        # Gravity force
        gravity_force = np.array([0, 0, -mass * self.g])
        
        # Drag force
        velocity_magnitude = math.sqrt(vel.vx**2 + vel.vy**2 + vel.vz**2)
        if velocity_magnitude > 0:
            drag_force = -0.5 * self.air_density * self.cross_sectional_area * \
                        self.drag_coefficient * velocity_magnitude * \
                        np.array([vel.vx, vel.vy, vel.vz])
        else:
            drag_force = np.array([0, 0, 0])
        
        # Total force
        total_force = thrust_world + gravity_force + drag_force
        
        # Linear acceleration
        acceleration = total_force / mass if mass > 0 else np.array([0, 0, 0])
        
        # Update linear motion
        new_velocity = Velocity(
            vx=vel.vx + acceleration[0] * dt,
            vy=vel.vy + acceleration[1] * dt,
            vz=vel.vz + acceleration[2] * dt
        )
        
        new_position = Position(
            x=pos.x + vel.vx * dt + 0.5 * acceleration[0] * dt**2,
            y=pos.y + vel.vy * dt + 0.5 * acceleration[1] * dt**2,
            z=pos.z + vel.vz * dt + 0.5 * acceleration[2] * dt**2
        )
        
        # Angular dynamics (simplified)
        # Attitude control provides angular accelerations
        alpha = 10.0  # Angular damping coefficient
        angular_acceleration = Attitude(
            roll=attitude_control.roll - alpha * ang_vel.roll,
            pitch=attitude_control.pitch - alpha * ang_vel.pitch,
            yaw=attitude_control.yaw - alpha * ang_vel.yaw
        )
        
        # Update angular motion
        new_angular_velocity = Attitude(
            roll=ang_vel.roll + angular_acceleration.roll * dt,
            pitch=ang_vel.pitch + angular_acceleration.pitch * dt,
            yaw=ang_vel.yaw + angular_acceleration.yaw * dt
        )
        
        new_attitude = Attitude(
            roll=att.roll + ang_vel.roll * dt + 0.5 * angular_acceleration.roll * dt**2,
            pitch=att.pitch + ang_vel.pitch * dt + 0.5 * angular_acceleration.pitch * dt**2,
            yaw=att.yaw + ang_vel.yaw * dt + 0.5 * angular_acceleration.yaw * dt**2
        )
        
        # Fuel consumption
        fuel_consumed = thrust_vector.magnitude * self.fuel_consumption_rate * dt
        new_fuel = max(0, fuel - fuel_consumed)
        new_mass = self.dry_mass + new_fuel
        
        # Create new state
        self.state = RocketState(
            position=new_position,
            velocity=new_velocity,
            attitude=new_attitude,
            angular_velocity=new_angular_velocity,
            mass=new_mass,
            fuel_remaining=new_fuel
        )
        
        return self.state
    
    def _body_to_world(self, vector_body: np.ndarray, attitude: Attitude) -> np.ndarray:
        """Transform vector from body frame to world frame"""
        # Rotation matrices for roll, pitch, yaw
        cos_r, sin_r = math.cos(attitude.roll), math.sin(attitude.roll)
        cos_p, sin_p = math.cos(attitude.pitch), math.sin(attitude.pitch)
        cos_y, sin_y = math.cos(attitude.yaw), math.sin(attitude.yaw)
        
        # Combined rotation matrix (ZYX Euler angles)
        R = np.array([
            [cos_y*cos_p, cos_y*sin_p*sin_r - sin_y*cos_r, cos_y*sin_p*cos_r + sin_y*sin_r],
            [sin_y*cos_p, sin_y*sin_p*sin_r + cos_y*cos_r, sin_y*sin_p*cos_r - cos_y*sin_r],
            [-sin_p, cos_p*sin_r, cos_p*cos_r]
        ])
        
        return R @ vector_body
    
    def get_thrust_to_weight_ratio(self) -> float:
        """Calculate current thrust-to-weight ratio"""
        if self.state.mass > 0:
            weight = self.state.mass * self.g
            return self.max_thrust / weight
        return 0.0
    
    def is_crashed(self) -> bool:
        """Check if rocket has crashed"""
        return self.state.position.z < 0
    
    def is_out_of_fuel(self) -> bool:
        """Check if rocket is out of fuel"""
        return self.state.fuel_remaining <= 0