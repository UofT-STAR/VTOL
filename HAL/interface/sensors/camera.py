from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List

class Camera(ABC):
    """Base interface for defining camera hardware class"""

    @abstractmethod
    def initialize(self) -> None:
        """Prepare the camera for operation."""
        pass

    @abstractmethod
    def capture(self) -> Dict[str, Any]:
        """Return the camera data in a standardized format."""
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """Safely release resources or power down."""
        pass