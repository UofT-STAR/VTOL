# VTOL - Thrust Vector Control System

A comprehensive thrust vector control system for vertical landing and takeoff rockets, featuring advanced PID-based control algorithms, realistic physics simulation, and sensor modeling.

## Features

### 🚀 Control System
- **PID Controllers**: Tunable proportional-integral-derivative controllers for precise control
- **Attitude Control**: Roll, pitch, and yaw stabilization using gyroscopic feedback
- **Thrust Vectoring**: Dynamic thrust direction control for precise maneuvering
- **Position Control**: 3D position holding and waypoint navigation
- **Landing Sequence**: Automated vertical landing with controlled descent

### 🔬 Simulation Environment
- **Physics Engine**: Realistic rocket dynamics including mass, inertia, and fuel consumption
- **Environmental Effects**: Wind, turbulence, and atmospheric density variations
- **Sensor Simulation**: IMU, GPS, and altimeter with realistic noise characteristics
- **Real-time Visualization**: Monitor flight parameters and control performance

### 📊 Key Capabilities
- Vertical takeoff and landing (VTOL) operations
- Hover and station-keeping
- Autonomous flight mission execution
- Configurable control parameters
- Comprehensive data logging and analysis

## Quick Start

### Prerequisites
```bash
pip install numpy
```

### Basic Test
```bash
python test_system.py
```

### Configuration
Edit `config.py` to adjust control gains, rocket parameters, and simulation settings:

```python
# Control system gains
CONTROL_GAINS = {
    'attitude': {
        'roll_gains': (2.0, 0.1, 0.5),    # (kp, ki, kd)
        'pitch_gains': (2.0, 0.1, 0.5),   
        'yaw_gains': (1.5, 0.05, 0.3)     
    },
    'position': {
        'altitude_gains': (1.0, 0.02, 0.8),
        'horizontal_gains': (0.5, 0.01, 0.3)
    }
}
```

## System Architecture

```
VTOL Control System
├── Control Module
│   ├── PID Controller          # Basic PID implementation
│   ├── Attitude Controller     # Roll/pitch/yaw control
│   └── Thrust Vector Controller # Main control system
├── Simulation Module
│   ├── Rocket Dynamics         # Physics simulation
│   ├── Environment            # Wind and atmospheric effects
│   └── Complete Simulation    # Integrated simulation
├── Sensors Module
│   ├── IMU Sensor             # Inertial measurement unit
│   ├── GPS Sensor             # Position measurement
│   └── Altimeter Sensor       # Altitude measurement
└── Examples
    └── Basic Simulation       # Demo flight scenario
```

## Control System Details

### Thrust Vector Control
The thrust vector controller manages the rocket's orientation and position by:
1. **Altitude Control**: Maintains desired height using vertical thrust
2. **Horizontal Control**: Uses thrust vectoring for lateral positioning
3. **Attitude Stabilization**: Coordinates thrust direction with attitude control
4. **Landing Sequence**: Executes controlled descent with precision landing

### PID Controller Features
- Configurable gains (Kp, Ki, Kd)
- Output limiting and saturation
- Anti-windup protection
- Real-time gain adjustment

### Mission Phases
1. **Takeoff**: Vertical ascent to target altitude
2. **Hover**: Station-keeping at specified position
3. **Landing**: Controlled descent with soft touchdown

## Technical Specifications

### Rocket Parameters
- **Mass**: 500 kg (dry) + 200 kg (fuel)
- **Thrust**: Up to 8000 N (thrust-to-weight ratio: ~1.2)
- **Dimensions**: 10m length × 1m diameter
- **Thrust Vector Angle**: ±15° maximum deflection

### Control Performance
- **Attitude Accuracy**: ±0.5° (1σ)
- **Position Accuracy**: ±1m horizontal, ±0.5m vertical
- **Landing Precision**: <0.5m CEP (circular error probable)
- **Update Rate**: 100 Hz control loop

### Sensor Specifications
- **IMU**: 0.01 rad attitude noise, 0.05 rad/s gyro noise
- **GPS**: 2-10m accuracy depending on satellite count
- **Altimeter**: 0.5m accuracy (barometric)

## Usage Examples

### Basic Control Test
```python
from src.control.thrust_vector_controller import ThrustVectorController, Position, Velocity
from src.control.attitude_controller import Attitude

# Initialize controller
controller = ThrustVectorController()

# Set target and current state
target = Position(x=0, y=0, z=50)  # Hover at 50m
current_pos = Position(x=0, y=0, z=0)
current_vel = Velocity(vx=0, vy=0, vz=0)
current_att = Attitude(roll=0, pitch=0, yaw=0)

# Update control
thrust_vector, attitude_control = controller.update(
    target, current_pos, current_vel, current_att, dt=0.01
)
```

### Landing Sequence
```python
# Initiate landing
controller.initiate_landing(target_altitude=0.0)

# Controller will now execute controlled descent
```

## Testing

Run the test suite to verify system functionality:
```bash
python tests/test_control.py
```

Test individual components:
```bash
python test_system.py
```

## Configuration Parameters

The system can be tuned via `config.py`:

- **Control Gains**: Adjust PID parameters for different flight characteristics
- **Physical Parameters**: Modify rocket mass, thrust, and dimensions
- **Sensor Noise**: Configure realistic sensor error models
- **Environmental**: Set wind and atmospheric conditions

## Performance Tuning

### Aggressive Control (High Performance)
```python
CONTROL_GAINS = {
    'attitude': {
        'roll_gains': (3.0, 0.2, 0.8),
        'pitch_gains': (3.0, 0.2, 0.8),
        'yaw_gains': (2.5, 0.1, 0.5)
    }
}
```

### Conservative Control (Smooth, Stable)
```python
CONTROL_GAINS = {
    'attitude': {
        'roll_gains': (1.5, 0.05, 0.3),
        'pitch_gains': (1.5, 0.05, 0.3),
        'yaw_gains': (1.0, 0.02, 0.2)
    }
}
```

## License

MIT License - see LICENSE file for details.

## Contributing

This project is part of the University of Toronto Space Team (UofT STAR) VTOL rocket development program. Contributions welcome for improvements to control algorithms, simulation fidelity, and system integration.
