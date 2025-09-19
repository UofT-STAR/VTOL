"""
Altimeter sensor simulation for precise altitude measurement
"""

import numpy as np
from typing import NamedTuple


class AltimeterReading(NamedTuple):
    """Altimeter sensor reading"""
    altitude: float  # meters above ground level
    pressure: float  # atmospheric pressure (Pa)


class AltimeterSensor:
    """
    Simulated barometric altimeter sensor
    Provides more accurate altitude than GPS
    """
    
    def __init__(self, 
                 accuracy: float = 0.5,  # meters
                 sea_level_pressure: float = 101325.0):  # Pa
        """
        Initialize altimeter sensor
        
        Args:
            accuracy: Altitude measurement accuracy in meters
            sea_level_pressure: Sea level atmospheric pressure in Pascals
        """
        self.accuracy = accuracy
        self.sea_level_pressure = sea_level_pressure
        
    def read(self, true_altitude: float) -> AltimeterReading:
        """
        Read altimeter sensor
        
        Args:
            true_altitude: True altitude above ground level
            
        Returns:
            Altimeter reading with noise
        """
        # Add measurement noise
        noise_std = self.accuracy / 3.0
        noisy_altitude = true_altitude + np.random.normal(0, noise_std)
        
        # Calculate atmospheric pressure based on altitude
        # Using barometric formula (simplified)
        pressure = self.sea_level_pressure * (1 - 0.0065 * true_altitude / 288.15) ** 5.255
        
        # Add small pressure noise
        pressure_noise = np.random.normal(0, 10.0)  # ±10 Pa
        noisy_pressure = pressure + pressure_noise
        
        return AltimeterReading(
            altitude=noisy_altitude,
            pressure=noisy_pressure
        )