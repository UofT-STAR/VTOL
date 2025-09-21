from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Altimeter(ABC):
    """Altimeter interface, to be implemented in hardware layer"""

    @abstractmethod
    def initialize(self)->None:
        """Initialise an instance of this altimeter"""
        pass

    @abstractmethod
    def read(self)->None:
        """Read sensor data"""
        pass

    @abstractmethod
    def shutdown(self)->None:
        """Safely shutdown the altimeter"""
        pass