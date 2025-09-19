"""
Complete VTOL simulation system
Integrates control, dynamics, sensors, and environment
"""

import time
import math
import numpy as np
from typing import List, Dict, Optional, Callable
from dataclasses import dataclass

from .rocket_dynamics import RocketDynamics, RocketState
from .environment import Environment, Wind
from ..control.thrust_vector_controller import ThrustVectorController, Position, Velocity
from ..control.attitude_controller import Attitude
from ..sensors.imu import IMUSensor, IMUReading
from ..sensors.gps import GPSSensor, GPSReading
from ..sensors.altimeter import AltimeterSensor, AltimeterReading


@dataclass
class SimulationConfig:
    """Simulation configuration parameters"""
    dt: float = 0.01  # Time step (seconds)
    max_time: float = 300.0  # Maximum simulation time (seconds)
    real_time_factor: float = 1.0  # Real-time scaling (1.0 = real-time)
    log_interval: float = 0.1  # Data logging interval (seconds)


@dataclass
class SimulationData:
    """Single timestep of simulation data"""
    time: float
    rocket_state: RocketState
    target_position: Position
    thrust_vector: tuple
    wind: Wind
    sensor_readings: Dict


class VTOLSimulation:
    """
    Complete VTOL rocket simulation
    Integrates all subsystems for realistic flight simulation
    """
    
    def __init__(self, config: Optional[SimulationConfig] = None):
        """
        Initialize VTOL simulation
        
        Args:
            config: Simulation configuration
        """
        self.config = config or SimulationConfig()
        
        # Initialize subsystems
        self.rocket = RocketDynamics()
        self.environment = Environment()
        self.controller = ThrustVectorController()
        
        # Initialize sensors
        self.imu = IMUSensor()
        self.gps = GPSSensor()
        self.altimeter = AltimeterSensor()
        
        # Simulation state
        self.time = 0.0
        self.data_log: List[SimulationData] = []
        self.last_log_time = 0.0
        
        # Mission parameters
        self.mission_target = Position(0, 0, 50)  # Default hover at 50m
        self.mission_phase = "takeoff"  # takeoff, hover, landing
        
    def reset(self, 
              initial_position: Position = Position(0, 0, 0),
              initial_velocity: Velocity = Velocity(0, 0, 0),
              initial_attitude: Attitude = Attitude(0, 0, 0)):
        """Reset simulation to initial state"""
        self.time = 0.0
        self.data_log.clear()
        self.last_log_time = 0.0
        self.mission_phase = "takeoff"
        
        self.rocket.reset(initial_position, initial_velocity, initial_attitude)
        self.controller.reset()
        
    def set_mission_target(self, target: Position):
        """Set mission target position"""
        self.mission_target = target
        
    def step(self) -> bool:
        """
        Execute one simulation step
        
        Returns:
            True if simulation should continue, False if ended
        """
        # Check termination conditions
        if self.time >= self.config.max_time:
            return False
            
        if self.rocket.is_crashed():
            print(f"Mission failed: Rocket crashed at t={self.time:.2f}s")
            return False
            
        # Get current rocket state
        rocket_state = self.rocket.state
        
        # Update environment
        wind = self.environment.update(self.config.dt, rocket_state.position.z)
        
        # Read sensors
        sensor_readings = self._read_sensors(rocket_state, wind)
        
        # Mission logic
        current_target = self._update_mission_logic(rocket_state)
        
        # Control system update
        thrust_vector, attitude_control = self.controller.update(
            target_position=current_target,
            current_position=sensor_readings['gps'].position if sensor_readings['gps'] else rocket_state.position,
            current_velocity=rocket_state.velocity,
            current_attitude=sensor_readings['imu'].attitude,
            dt=self.config.dt
        )
        
        # Update rocket dynamics
        new_state = self.rocket.update(thrust_vector, attitude_control, self.config.dt)
        
        # Log data
        if self.time - self.last_log_time >= self.config.log_interval:
            self._log_data(thrust_vector, wind, sensor_readings, current_target)
            self.last_log_time = self.time
            
        # Update time
        self.time += self.config.dt
        
        return True
    
    def _read_sensors(self, rocket_state: RocketState, wind: Wind) -> Dict:
        """Read all sensors with realistic noise and failures"""
        sensors = {}
        
        # IMU reading
        # Calculate acceleration including thrust and gravity
        acceleration = (0, 0, 0)  # Simplified for now
        sensors['imu'] = self.imu.read(
            rocket_state.attitude,
            rocket_state.angular_velocity,
            acceleration
        )
        
        # GPS reading (if available)
        if self.gps.is_available():
            sensors['gps'] = self.gps.read(rocket_state.position)
        else:
            sensors['gps'] = None
            
        # Altimeter reading
        sensors['altimeter'] = self.altimeter.read(rocket_state.position.z)
        
        return sensors
    
    def _update_mission_logic(self, rocket_state: RocketState) -> Position:
        """Update mission phase and return current target"""
        position = rocket_state.position
        velocity = rocket_state.velocity
        
        if self.mission_phase == "takeoff":
            # Takeoff until reaching target altitude
            if position.z >= self.mission_target.z - 2.0:
                self.mission_phase = "hover"
                print(f"Takeoff complete at t={self.time:.2f}s, altitude={position.z:.1f}m")
            return Position(0, 0, self.mission_target.z)
            
        elif self.mission_phase == "hover":
            # Hover at target position
            # Automatically start landing after some time or on command
            if self.time > 60.0:  # Start landing after 60 seconds
                self.mission_phase = "landing"
                self.controller.initiate_landing()
                print(f"Landing initiated at t={self.time:.2f}s")
            return self.mission_target
            
        elif self.mission_phase == "landing":
            # Landing phase
            if position.z <= 1.0 and abs(velocity.vz) < 0.5:
                print(f"Landing successful at t={self.time:.2f}s")
                return Position(position.x, position.y, 0)
            return Position(self.mission_target.x, self.mission_target.y, 0)
        
        return self.mission_target
    
    def _log_data(self, thrust_vector, wind: Wind, sensor_readings: Dict, target: Position):
        """Log simulation data"""
        data = SimulationData(
            time=self.time,
            rocket_state=self.rocket.state,
            target_position=target,
            thrust_vector=(thrust_vector.magnitude, thrust_vector.angle_x, thrust_vector.angle_y),
            wind=wind,
            sensor_readings=sensor_readings
        )
        self.data_log.append(data)
    
    def run(self, callback: Optional[Callable] = None) -> bool:
        """
        Run complete simulation
        
        Args:
            callback: Optional callback function called each step
            
        Returns:
            True if mission successful, False if failed
        """
        print(f"Starting VTOL simulation...")
        print(f"Target: {self.mission_target}")
        print(f"Thrust-to-weight ratio: {self.rocket.get_thrust_to_weight_ratio():.2f}")
        
        start_time = time.time()
        
        while self.step():
            # Real-time scaling
            if self.config.real_time_factor > 0:
                elapsed = time.time() - start_time
                sim_time = self.time / self.config.real_time_factor
                if elapsed < sim_time:
                    time.sleep(sim_time - elapsed)
            
            # Call callback if provided
            if callback:
                callback(self)
                
        # Check final state
        final_state = self.rocket.state
        if (final_state.position.z <= 1.0 and 
            abs(final_state.velocity.vz) < 1.0 and
            not self.rocket.is_crashed()):
            print(f"Mission successful! Final altitude: {final_state.position.z:.2f}m")
            return True
        else:
            print(f"Mission failed! Final state: {final_state}")
            return False
    
    def get_summary(self) -> Dict:
        """Get simulation summary statistics"""
        if not self.data_log:
            return {}
            
        final_data = self.data_log[-1]
        max_altitude = max(d.rocket_state.position.z for d in self.data_log)
        
        return {
            'total_time': self.time,
            'final_position': final_data.rocket_state.position,
            'final_velocity': final_data.rocket_state.velocity,
            'max_altitude': max_altitude,
            'fuel_remaining': final_data.rocket_state.fuel_remaining,
            'mission_phase': self.mission_phase,
            'data_points': len(self.data_log)
        }