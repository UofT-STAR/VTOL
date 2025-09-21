"""
This class defines the hardware contract for
the temperature and humidity sensor
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class THD(ABC):

    @abstractmethod
    def initialize(self)->None:
        """Prepare the sensor for operation"""
        pass

    @abstractmethod
    def read(self)->Dict[str, Any]:
        """Return the sensor input in a standardised format"""
        pass

    @abstractmethod
    def shutdown(self)->None:
        """Safely release resources and power off"""
        pass