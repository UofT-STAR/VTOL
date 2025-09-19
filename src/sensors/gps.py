"""
GPS sensor simulation for position measurement
"""

import numpy as np
from typing import NamedTuple
from ..control.thrust_vector_controller import Position


class GPSReading(NamedTuple):
    """GPS sensor reading"""
    position: Position
    accuracy: float  # meters (estimated accuracy)
    satellites: int  # number of satellites


class GPSSensor:
    """
    Simulated GPS sensor for position measurement
    Includes realistic accuracy and availability characteristics
    """
    
    def __init__(self, 
                 base_accuracy: float = 2.0,  # meters
                 max_accuracy: float = 10.0):  # meters
        """
        Initialize GPS sensor
        
        Args:
            base_accuracy: Best-case accuracy in meters
            max_accuracy: Worst-case accuracy in meters
        """
        self.base_accuracy = base_accuracy
        self.max_accuracy = max_accuracy
        
    def read(self, true_position: Position) -> GPSReading:
        """
        Read GPS sensor with realistic accuracy
        
        Args:
            true_position: True position of the rocket
            
        Returns:
            Noisy GPS reading
        """
        # Simulate varying GPS accuracy based on satellite count
        satellites = np.random.randint(4, 12)
        
        # Better accuracy with more satellites
        if satellites >= 8:
            accuracy = self.base_accuracy
        elif satellites >= 6:
            accuracy = self.base_accuracy * 1.5
        else:
            accuracy = self.base_accuracy * 3.0
            
        accuracy = min(accuracy, self.max_accuracy)
        
        # Add GPS noise based on accuracy
        noise_std = accuracy / 3.0  # 99.7% of readings within accuracy
        
        noisy_position = Position(
            x=true_position.x + np.random.normal(0, noise_std),
            y=true_position.y + np.random.normal(0, noise_std),
            z=true_position.z + np.random.normal(0, noise_std * 1.5)  # Altitude less accurate
        )
        
        return GPSReading(
            position=noisy_position,
            accuracy=accuracy,
            satellites=satellites
        )
    
    def is_available(self) -> bool:
        """Check if GPS is available (simulates signal loss)"""
        # 95% availability
        return np.random.random() > 0.05