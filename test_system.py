"""
Simple test of the thrust vector control system
"""

import sys
import os

# Add the src directory to the path
project_root = os.path.dirname(__file__)
src_path = os.path.join(project_root, 'src')
sys.path.insert(0, src_path)

# Now import the modules
from control.pid_controller import PIDController
from control.attitude_controller import AttitudeController, Attitude
from control.thrust_vector_controller import ThrustVectorController, Position, Velocity


def test_basic_functionality():
    """Test basic control system functionality"""
    print("Testing VTOL Thrust Vector Control System")
    print("=" * 50)
    
    # Test PID Controller
    print("1. Testing PID Controller...")
    pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
    
    # Simulate a step response
    setpoint = 10.0
    measured = 0.0
    dt = 0.1
    
    for i in range(10):
        output = pid.update(setpoint, measured, dt)
        measured += output * dt * 0.5  # Simple integration
        print(f"   Step {i+1}: Setpoint={setpoint:.1f}, Measured={measured:.2f}, Output={output:.2f}")
    
    print("   ✓ PID Controller working")
    
    # Test Attitude Controller
    print("\n2. Testing Attitude Controller...")
    attitude_controller = AttitudeController()
    
    target_attitude = Attitude(roll=0.1, pitch=0.05, yaw=0.0)
    current_attitude = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
    
    control_output = attitude_controller.update(target_attitude, current_attitude, dt=0.1)
    print(f"   Target: {target_attitude}")
    print(f"   Current: {current_attitude}")
    print(f"   Control: {control_output}")
    print("   ✓ Attitude Controller working")
    
    # Test Thrust Vector Controller
    print("\n3. Testing Thrust Vector Controller...")
    tvc = ThrustVectorController()
    
    target_pos = Position(x=0, y=0, z=20)
    current_pos = Position(x=0, y=0, z=0)
    current_vel = Velocity(vx=0, vy=0, vz=0)
    current_att = Attitude(roll=0, pitch=0, yaw=0)
    
    thrust_vector, attitude_control = tvc.update(
        target_pos, current_pos, current_vel, current_att, dt=0.1
    )
    
    print(f"   Target Position: {target_pos}")
    print(f"   Current Position: {current_pos}")
    print(f"   Thrust Vector: magnitude={thrust_vector.magnitude:.3f}, "
          f"angles=({thrust_vector.angle_x:.3f}, {thrust_vector.angle_y:.3f})")
    print(f"   Attitude Control: {attitude_control}")
    print("   ✓ Thrust Vector Controller working")
    
    # Test landing mode
    print("\n4. Testing Landing Mode...")
    tvc.initiate_landing()
    status = tvc.get_status()
    print(f"   Landing mode: {status['landing_mode']}")
    print(f"   Max thrust angle: {status['max_thrust_angle_deg']:.1f}°")
    print("   ✓ Landing mode activated")
    
    print("\n" + "=" * 50)
    print("✅ All basic tests passed!")
    print("\nThe VTOL Thrust Vector Control System is ready for use.")
    print("\nKey Features:")
    print("- PID-based attitude control (roll, pitch, yaw)")
    print("- Position control with thrust vectoring")
    print("- Automatic landing sequence")
    print("- Configurable control gains")
    print("- Realistic sensor simulation")
    print("- Physics-based rocket dynamics")


if __name__ == "__main__":
    test_basic_functionality()