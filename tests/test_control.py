"""
Unit tests for VTOL control system
"""

import unittest
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from control.pid_controller import PIDController
from control.attitude_controller import AttitudeController, Attitude
from control.thrust_vector_controller import ThrustVectorController, Position, Velocity


class TestPIDController(unittest.TestCase):
    """Test PID controller functionality"""
    
    def test_proportional_control(self):
        """Test proportional control only"""
        pid = PIDController(kp=1.0, ki=0.0, kd=0.0)
        
        # Test step response
        output = pid.update(setpoint=10.0, measured_value=5.0, dt=0.1)
        self.assertEqual(output, 5.0)  # kp * error = 1.0 * 5.0
        
    def test_output_limits(self):
        """Test output limiting"""
        pid = PIDController(kp=10.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
        
        output = pid.update(setpoint=10.0, measured_value=0.0, dt=0.1)
        self.assertEqual(output, 1.0)  # Limited to max
        
        output = pid.update(setpoint=0.0, measured_value=10.0, dt=0.1)
        self.assertEqual(output, -1.0)  # Limited to min


class TestAttitudeController(unittest.TestCase):
    """Test attitude controller"""
    
    def test_initialization(self):
        """Test attitude controller initialization"""
        controller = AttitudeController()
        self.assertIsNotNone(controller.roll_controller)
        self.assertIsNotNone(controller.pitch_controller)
        self.assertIsNotNone(controller.yaw_controller)
        
    def test_attitude_control(self):
        """Test attitude control update"""
        controller = AttitudeController()
        
        target = Attitude(roll=0.1, pitch=0.0, yaw=0.0)
        current = Attitude(roll=0.0, pitch=0.0, yaw=0.0)
        
        output = controller.update(target, current, dt=0.1)
        self.assertIsInstance(output, Attitude)
        self.assertGreater(output.roll, 0)  # Should provide positive roll control


class TestThrustVectorController(unittest.TestCase):
    """Test thrust vector controller"""
    
    def test_initialization(self):
        """Test thrust vector controller initialization"""
        controller = ThrustVectorController()
        self.assertIsNotNone(controller.attitude_controller)
        self.assertIsNotNone(controller.altitude_controller)
        
    def test_control_update(self):
        """Test control system update"""
        controller = ThrustVectorController()
        
        target_pos = Position(x=0, y=0, z=10)
        current_pos = Position(x=0, y=0, z=5)
        current_vel = Velocity(vx=0, vy=0, vz=0)
        current_att = Attitude(roll=0, pitch=0, yaw=0)
        
        thrust_vector, attitude_control = controller.update(
            target_pos, current_pos, current_vel, current_att, dt=0.1
        )
        
        self.assertGreater(thrust_vector.magnitude, 0)  # Should provide thrust
        

if __name__ == '__main__':
    unittest.main()