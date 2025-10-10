from interface import Altimeter
from mpu6050 import mpu6050
import time

class MPU6050Altimeter(Altimeter):

    def __init__(self, address: int = 0x68):
        self.address = address
        self.sensor = None
        self.initialized = False
        self.altitude = 0.0
        self.last_time = None
        self.vertical_velocity = 0.0

    def initialize(self) -> None:
        """Initialize the MPU6050 sensor"""
        self.sensor = mpu6050(self.address)
        self.initialized = True
        self.last_time = time.time()
        print("MPU6050 initialized.")

    def read(self) -> dict:
        """Estimate relative altitude using acceleration integration"""
        if not self.initialized:
            raise RuntimeError("Sensor not initialized. Call initialize() first.")

        accel_data = self.sensor.get_accel_data()
        z_accel = accel_data['z'] - 9.81  # subtract gravity (m/s^2)
        
        # integrate acceleration to get velocity, then altitude
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.vertical_velocity += z_accel * dt
        self.altitude += self.vertical_velocity * dt

        return {
            'accel_z': z_accel,
            'velocity_z': self.vertical_velocity,
            'altitude_estimate': self.altitude
        }

    def shutdown(self) -> None:
        """Safely shutdown the altimeter"""
        self.sensor = None
        self.initialized = False
        print("MPU6050 shut down.")