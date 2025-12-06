from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Any, Dict
import smbus2
import time


class MS5611Base(ABC):
    """Abstract base class defining a unified interface for MS5611 sensors."""

    CMD_RESET = 0x1E
    CMD_CONVERT_D1 = 0x48   # Pressure, OSR=4096
    CMD_CONVERT_D2 = 0x58   # Temperature, OSR=4096
    CMD_ADC_READ = 0x00

    PROM_BASE_ADDR = 0xA2   # First calibration address (A2 .. AE)

    def __init__(self, bus_id: int = 1, address: int = 0x76) -> None:
        """Prepare the sensor for operation."""
        self.bus_id = bus_id
        self.address = address
        self._initialized = False
        self.bus = None
        self.calibration = {}

    # ------------------------------------------------------------
    @abstractmethod
    def initialize(self) -> None:
        """
        Prepare the sensor for operation.
        Reads calibration coefficients from PROM.
        """
        self.bus = smbus2.SMBus(self.bus_id)

        self.bus.write_byte(self.address, self.CMD_RESET)
        time.sleep(0.003)

        self.calibration = {}
        for i in range(6):
            addr = self.PROM_BASE_ADDR + (i * 2)
            data = self.bus.read_i2c_block_data(self.address, addr, 2)
            self.calibration[f"C{i+1}"] = (data[0] << 8) | data[1]

        self._initialized = True

    # ------------------------------------------------------------
    @abstractmethod
    def read(self) -> Dict[str, Any]:
        """
        Return sensor readings in a standardized format.
        Must return:
            - temperature (°C)
            - pressure (hPa)
        """

        self.bus.write_byte(self.address, self.CMD_CONVERT_D1)
        time.sleep(0.01)
        d1 = self._read_adc()

        self.bus.write_byte(self.address, self.CMD_CONVERT_D2)
        time.sleep(0.01)
        d2 = self._read_adc()

        c = self.calibration

        dT = d2 - c["C5"] * 256
        temp = 2000 + (dT * c["C6"]) / 8388608
        off = (c["C2"] * 65536) + (c["C4"] * dT) / 128
        sens = (c["C1"] * 32768) + (c["C3"] * dT) / 256

        pressure = ((d1 * sens / 2097152) - off) / 32768
        pressure_hpa = pressure / 100

        return {
            "temperature": float(temp / 100),  # °C
            "pressure": float(pressure_hpa)    # hPa
        }

    # ------------------------------------------------------------
    @abstractmethod
    def shutdown(self) -> None:
        """Safely power down the sensor."""
        if self.bus is not None:
            self.bus.write_byte(self.address, self.CMD_RESET)
            self.bus.close()

        self.bus = None
        self.calibration = {}
        self._initialized = False

    # ------------------------------------------------------------
    def _read_adc(self) -> int:
        """Read 24-bit ADC result."""
        data = self.bus.read_i2c_block_data(self.address, self.CMD_ADC_READ, 3)
        return (data[0] << 16) | (data[1] << 8) | data[2]
