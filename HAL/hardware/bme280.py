"""bme280 abstraction"""
from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List
import smbus2
import bme280

class THD(ABC):

    CTRL_MEAS_REG = 0xF4

    def __init__ (self, bus_id: int = 1, address: int = 0x76)->None:
        """Prepare the sensor for operation"""
        self.bus_id = bus_id
        self.address = address
        self._initialized = False

    @abstractmethod
    def initialize (self)->None:
        """Prepare the sensor for operation"""
        self.bus = smbus2.SMBus(self.bus_id)
        self.calibration_params = bme280.load_calibration_params(self.bus, self.address)
        self._initialized = True

    @abstractmethod
    def read (self)->Dict[str, Any]:
        """Return the sensor input in a standardised format"""

        data = bme280.sample(self.bus, self.address, self.calibration_params)

        return {
            "temperature": float(data.temperature), # in degrees Celsius
            "humidity": float(data.humidity), # in % relative humidity
            "pressure": float(data.pressure)  # in hPa
        }

    @abstractmethod
    def shutdown (self)->None:
        """Safely release resources and power off"""
        if self.bus is not None:
            self.bus.write_byte_data(self.address, self.CTRL_MEAS_REG, 0x00)
        self.bus = None
        self.cal = None
        self._initialized = False

        self.bus.write_byte_data(self.address, CTRL_MEAS_REG, 0x00)
