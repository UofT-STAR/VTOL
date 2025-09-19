"""
IMU (Inertial Measurement Unit) sensor simulation
Provides attitude and angular velocity measurements
"""

import math
import numpy as np
from typing import NamedTuple
from ..control.attitude_controller import Attitude


class IMUReading(NamedTuple):
    """IMU sensor reading"""
    attitude: Attitude
    angular_velocity: Attitude  # rad/s
    acceleration: tuple  # m/s^2 (x, y, z)


class IMUSensor:
    """
    Simulated IMU sensor for attitude measurement
    Includes realistic noise and bias characteristics
    """
    
    def __init__(self, 
                 attitude_noise_std: float = 0.01,  # radians
                 gyro_noise_std: float = 0.05,      # rad/s
                 accel_noise_std: float = 0.1):     # m/s^2
        """
        Initialize IMU sensor
        
        Args:
            attitude_noise_std: Standard deviation of attitude noise
            gyro_noise_std: Standard deviation of gyroscope noise
            accel_noise_std: Standard deviation of accelerometer noise
        """
        self.attitude_noise_std = attitude_noise_std
        self.gyro_noise_std = gyro_noise_std
        self.accel_noise_std = accel_noise_std
        
        # Sensor biases (simulate realistic sensor errors)
        self.attitude_bias = Attitude(
            roll=np.random.normal(0, 0.005),
            pitch=np.random.normal(0, 0.005),
            yaw=np.random.normal(0, 0.01)
        )
        
        self.gyro_bias = Attitude(
            roll=np.random.normal(0, 0.02),
            pitch=np.random.normal(0, 0.02),
            yaw=np.random.normal(0, 0.02)
        )
        
    def read(self, true_attitude: Attitude, true_angular_velocity: Attitude, 
             true_acceleration: tuple) -> IMUReading:
        """
        Read IMU sensor with noise
        
        Args:
            true_attitude: True attitude of the rocket
            true_angular_velocity: True angular velocity
            true_acceleration: True acceleration
            
        Returns:
            Noisy IMU reading
        """
        # Add noise and bias to attitude
        noisy_attitude = Attitude(
            roll=true_attitude.roll + self.attitude_bias.roll + 
                 np.random.normal(0, self.attitude_noise_std),
            pitch=true_attitude.pitch + self.attitude_bias.pitch + 
                  np.random.normal(0, self.attitude_noise_std),
            yaw=true_attitude.yaw + self.attitude_bias.yaw + 
                np.random.normal(0, self.attitude_noise_std)
        )
        
        # Add noise and bias to gyroscope
        noisy_angular_velocity = Attitude(
            roll=true_angular_velocity.roll + self.gyro_bias.roll + 
                 np.random.normal(0, self.gyro_noise_std),
            pitch=true_angular_velocity.pitch + self.gyro_bias.pitch + 
                  np.random.normal(0, self.gyro_noise_std),
            yaw=true_angular_velocity.yaw + self.gyro_bias.yaw + 
                np.random.normal(0, self.gyro_noise_std)
        )
        
        # Add noise to accelerometer
        noisy_acceleration = (
            true_acceleration[0] + np.random.normal(0, self.accel_noise_std),
            true_acceleration[1] + np.random.normal(0, self.accel_noise_std),
            true_acceleration[2] + np.random.normal(0, self.accel_noise_std)
        )
        
        return IMUReading(
            attitude=noisy_attitude,
            angular_velocity=noisy_angular_velocity,
            acceleration=noisy_acceleration
        )
    
    def calibrate(self):
        """Simulate sensor calibration (reduces bias)"""
        # Reduce biases by 80% after calibration
        self.attitude_bias = Attitude(
            roll=self.attitude_bias.roll * 0.2,
            pitch=self.attitude_bias.pitch * 0.2,
            yaw=self.attitude_bias.yaw * 0.2
        )
        
        self.gyro_bias = Attitude(
            roll=self.gyro_bias.roll * 0.2,
            pitch=self.gyro_bias.pitch * 0.2,
            yaw=self.gyro_bias.yaw * 0.2
        )