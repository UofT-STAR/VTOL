"""
VTOL configuration parameters
Adjust these values to tune the control system performance
"""

# Rocket physical parameters
ROCKET_CONFIG = {
    'dry_mass': 500.0,          # kg - Empty rocket mass
    'fuel_mass': 200.0,         # kg - Fuel mass
    'max_thrust': 8000.0,       # N - Maximum thrust force
    'length': 10.0,             # m - Rocket length
    'diameter': 1.0,            # m - Rocket diameter
    'drag_coefficient': 0.5     # Aerodynamic drag coefficient
}

# Control system gains
CONTROL_GAINS = {
    # Attitude control (roll, pitch, yaw)
    'attitude': {
        'roll_gains': (2.0, 0.1, 0.5),    # (kp, ki, kd)
        'pitch_gains': (2.0, 0.1, 0.5),   # (kp, ki, kd)
        'yaw_gains': (1.5, 0.05, 0.3)     # (kp, ki, kd)
    },
    
    # Position control
    'position': {
        'altitude_gains': (1.0, 0.02, 0.8),   # (kp, ki, kd)
        'horizontal_gains': (0.5, 0.01, 0.3)  # (kp, ki, kd)
    },
    
    # Thrust vector limits
    'max_thrust_angle_deg': 15.0  # Maximum thrust vector angle in degrees
}

# Sensor characteristics
SENSOR_CONFIG = {
    'imu': {
        'attitude_noise_std': 0.01,     # radians
        'gyro_noise_std': 0.05,         # rad/s
        'accel_noise_std': 0.1          # m/s^2
    },
    
    'gps': {
        'base_accuracy': 2.0,           # meters
        'max_accuracy': 10.0            # meters
    },
    
    'altimeter': {
        'accuracy': 0.5                 # meters
    }
}

# Environment parameters
ENVIRONMENT_CONFIG = {
    'base_wind_speed': 5.0,         # m/s
    'wind_variability': 2.0,        # m/s
    'turbulence_intensity': 0.5     # m/s
}

# Simulation parameters
SIMULATION_CONFIG = {
    'dt': 0.01,                     # Time step (seconds)
    'max_time': 300.0,              # Maximum simulation time (seconds)
    'real_time_factor': 1.0,        # Real-time scaling
    'log_interval': 0.1             # Data logging interval (seconds)
}

# Mission parameters
MISSION_CONFIG = {
    'default_target_altitude': 50.0,    # meters
    'hover_duration': 30.0,             # seconds
    'landing_descent_rate': -2.0,       # m/s
    'landing_approach_altitude': 10.0   # meters
}