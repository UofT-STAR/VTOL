from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List
import serial
import pynmea2
import time

class Gps(ABC):
    """Base class for defining gps hardware"""

    @abstractmethod
    def initialise(self) -> None:
        """Prepare the GPS sensor for operation"""
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print("NEO-6M GPS initialized")
        except Exception as e:
            print(f"GPS initialization failed: {e}")

    @abstractmethod
    def read(self) -> Dict[str, Any]:
        """Return the sensor data in a standardised format"""
        if not self.serial:
            raise RuntimeError("GPS not initialized")

        data = {
            'latitude': None,
            'longitude': None,
            'satellites': 0,
            'fix': False
        }

        try:
            line = self.serial.readline().decode('ascii', errors='ignore').strip()

            if line.startswith('$GPGGA'):
                parts = line.split(',')

                if parts[6] and parts[6] != '0':  # Has fix
                    data['fix'] = True
                    data['satellites'] = int(parts[7]) if parts[7] else 0

                    # Parse latitude
                    if parts[2]:
                        lat = float(parts[2])
                        data['latitude'] = int(lat / 100) + (lat % 100) / 60)
                        if parts[3] == 'S':
                            data['latitude'] *= -1

                    # Parse longitude
                    if parts[4]:
                        lon = float(parts[4])
                        data['longitude'] = int(lon / 100) + (lon % 100) / 60)
                        if parts[5] == 'W':
                            data['longitude'] *= -1

        except Exception(BaseException):
            pass  # Silent fail on errors

        return data

    @abstractmethod
    def shutdown(self) -> None:
        """Safely shutdown gps sensor"""
        if self.serial:
            self.serial.close()
