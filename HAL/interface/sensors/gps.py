from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Gps(ABC):
    """Base class for defining gps hardware"""

    @abstractmethod
    def initialise(self) -> None:
        """Prepare the GPS sensor for operation"""
        pass

    @abstractmethod
    def read(self) -> Dict[str, Any]:
        """Return the sensor data in a standardised format"""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Safely shutdown gps sensor"""
        pass