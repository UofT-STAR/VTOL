from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Accelerometer(ABC):
    """Base interface for defining accelerometer sensor"""

    @abstractmethod
    def initialise(self)->None:
        """Prepare sensor for operation"""
        pass

    @abstractmethod
    def read(self)->Dict[str, Any]:
        """Return sensor information in a standardised format"""
        pass

    @abstractmethod
    def shutdown(self)->None:
        """Safely release resources and power off"""
        pass