from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Radar(ABC):
    """Base class for defining Radar hardware class"""

    @abstractmethod
    def initialize(self) -> None:
        """Prepare the sensor for operation"""
        pass

    @abstractmethod
    def read(self)-> Dict[str, Any]:
        """Return the sensor data in a standardised format"""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Safely release resources and power off"""
        pass