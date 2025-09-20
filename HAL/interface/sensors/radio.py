from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Radio(ABC):
    """Base interface for defining radio hardware class"""

    @abstractmethod
    def initialize(self)->None:
        """Prepare the receiver and transmitter for operation"""
        pass

    @abstractmethod
    def receive(self)->Dict[str, Any]:
        """Return the sensor data in a standardised format"""
        pass

    @abstractmethod
    def transmit(self)->Dict[str, Any]:
        """Send the sensor data in a standardised format"""
        pass

    @abstractmethod
    def shutdown(self)-> None:
        """Safely release resources and power off"""
        pass