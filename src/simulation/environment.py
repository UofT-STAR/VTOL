"""
Environment simulation for VTOL rocket
Models atmospheric conditions and external disturbances
"""

import math
import numpy as np
from typing import NamedTuple


class Wind(NamedTuple):
    """Wind conditions"""
    velocity_x: float  # m/s
    velocity_y: float  # m/s
    velocity_z: float  # m/s (updraft/downdraft)


class Environment:
    """
    Environmental conditions simulation
    Includes wind, atmospheric density variation, and disturbances
    """
    
    def __init__(self,
                 base_wind_speed: float = 5.0,    # m/s
                 wind_variability: float = 2.0,   # m/s
                 turbulence_intensity: float = 0.5):  # m/s
        """
        Initialize environment
        
        Args:
            base_wind_speed: Average wind speed
            wind_variability: Wind speed variation
            turbulence_intensity: Turbulence strength
        """
        self.base_wind_speed = base_wind_speed
        self.wind_variability = wind_variability
        self.turbulence_intensity = turbulence_intensity
        
        # Wind direction (changes slowly over time)
        self.wind_direction = np.random.uniform(0, 2 * math.pi)
        self.wind_direction_rate = 0.01  # rad/s
        
    def update(self, dt: float, altitude: float) -> Wind:
        """
        Update environmental conditions
        
        Args:
            dt: Time step
            altitude: Current altitude
            
        Returns:
            Current wind conditions
        """
        # Update wind direction
        self.wind_direction += self.wind_direction_rate * dt
        
        # Wind speed varies with altitude and time
        altitude_factor = min(1.0, altitude / 100.0)  # Wind increases with altitude
        wind_speed = self.base_wind_speed * altitude_factor + \
                    np.random.normal(0, self.wind_variability)
        
        # Wind components
        wind_x = wind_speed * math.cos(self.wind_direction)
        wind_y = wind_speed * math.sin(self.wind_direction)
        
        # Vertical wind (updrafts/downdrafts)
        wind_z = np.random.normal(0, self.turbulence_intensity * 0.5)
        
        # Add turbulence
        turbulence_x = np.random.normal(0, self.turbulence_intensity)
        turbulence_y = np.random.normal(0, self.turbulence_intensity)
        turbulence_z = np.random.normal(0, self.turbulence_intensity * 0.3)
        
        return Wind(
            velocity_x=wind_x + turbulence_x,
            velocity_y=wind_y + turbulence_y,
            velocity_z=wind_z + turbulence_z
        )
    
    def get_air_density(self, altitude: float) -> float:
        """
        Calculate air density at given altitude
        
        Args:
            altitude: Altitude in meters
            
        Returns:
            Air density in kg/m^3
        """
        # Standard atmosphere model (simplified)
        sea_level_density = 1.225  # kg/m^3
        scale_height = 8400.0  # m
        
        return sea_level_density * math.exp(-altitude / scale_height)
    
    def apply_wind_force(self, wind: Wind, rocket_velocity: tuple, 
                        cross_sectional_area: float, drag_coefficient: float) -> tuple:
        """
        Calculate wind force on rocket
        
        Args:
            wind: Wind conditions
            rocket_velocity: Rocket velocity (vx, vy, vz)
            cross_sectional_area: Rocket cross-sectional area
            drag_coefficient: Drag coefficient
            
        Returns:
            Wind force (fx, fy, fz)
        """
        # Relative velocity of rocket with respect to air
        relative_velocity = (
            rocket_velocity[0] - wind.velocity_x,
            rocket_velocity[1] - wind.velocity_y,
            rocket_velocity[2] - wind.velocity_z
        )
        
        # Air density (assume sea level for simplicity)
        air_density = 1.225
        
        # Dynamic pressure
        rel_speed = math.sqrt(sum(v**2 for v in relative_velocity))
        if rel_speed > 0:
            dynamic_pressure = 0.5 * air_density * rel_speed**2
            
            # Force direction opposite to relative velocity
            force_magnitude = dynamic_pressure * cross_sectional_area * drag_coefficient
            force_direction = tuple(-v / rel_speed for v in relative_velocity)
            
            return tuple(force_magnitude * d for d in force_direction)
        
        return (0.0, 0.0, 0.0)