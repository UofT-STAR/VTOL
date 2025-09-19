"""
Advanced VTOL mission demonstration
Shows a complete mission profile including takeoff, waypoint navigation, and precision landing
"""

import sys
import os
import time
import math

# Add the src directory to the path
project_root = os.path.dirname(__file__)
src_path = os.path.join(project_root, '..', 'src')
sys.path.insert(0, src_path)

from control.thrust_vector_controller import ThrustVectorController, Position, Velocity
from control.attitude_controller import Attitude
from simulation.rocket_dynamics import RocketDynamics
from sensors.imu import IMUSensor
from sensors.gps import GPSSensor
from sensors.altimeter import AltimeterSensor


def run_mission_simulation():
    """Run a complete VTOL mission simulation"""
    
    print("🚀 VTOL Mission Simulation")
    print("=" * 60)
    
    # Initialize systems
    rocket = RocketDynamics()
    controller = ThrustVectorController()
    imu = IMUSensor()
    gps = GPSSensor()
    altimeter = AltimeterSensor()
    
    # Mission waypoints
    waypoints = [
        Position(x=0, y=0, z=30),    # Takeoff to 30m
        Position(x=10, y=0, z=30),   # Move east 10m
        Position(x=10, y=10, z=30),  # Move north 10m
        Position(x=0, y=10, z=30),   # Move west 10m
        Position(x=0, y=0, z=30),    # Return to start
        Position(x=0, y=0, z=0)      # Land
    ]
    
    current_waypoint = 0
    mission_time = 0.0
    dt = 0.1  # 10 Hz control rate
    
    print(f"Mission Profile: {len(waypoints)} waypoints")
    for i, wp in enumerate(waypoints):
        print(f"  {i+1}. Position({wp.x:4.1f}, {wp.y:4.1f}, {wp.z:4.1f})")
    print()
    
    # Mission loop
    print("Mission Status:")
    print("Time(s) | Alt(m) | Pos(m)     | Vel(m/s)   | Fuel(kg) | Waypoint")
    print("-" * 60)
    
    while current_waypoint < len(waypoints) and mission_time < 300:
        # Current target
        target = waypoints[current_waypoint]
        
        # Get rocket state
        rocket_state = rocket.state
        
        # Read sensors (with noise)
        imu_reading = imu.read(
            rocket_state.attitude,
            rocket_state.angular_velocity,
            (0, 0, -9.81)  # Simplified acceleration
        )
        
        if gps.is_available():
            gps_reading = gps.read(rocket_state.position)
            current_pos = gps_reading.position
        else:
            current_pos = rocket_state.position  # Fallback to true position
            
        altimeter_reading = altimeter.read(rocket_state.position.z)
        
        # Control system update
        thrust_vector, attitude_control = controller.update(
            target_position=target,
            current_position=current_pos,
            current_velocity=rocket_state.velocity,
            current_attitude=imu_reading.attitude,
            dt=dt
        )
        
        # Update rocket dynamics
        rocket.update(thrust_vector, attitude_control, dt)
        
        # Check waypoint completion
        pos_error = math.sqrt(
            (rocket_state.position.x - target.x)**2 +
            (rocket_state.position.y - target.y)**2 +
            (rocket_state.position.z - target.z)**2
        )
        
        if pos_error < 2.0:  # Within 2m of target
            if current_waypoint == len(waypoints) - 1:
                # Final landing waypoint
                if rocket_state.position.z < 1.0 and abs(rocket_state.velocity.vz) < 0.5:
                    print(f"{mission_time:7.1f} | Landing successful!")
                    break
            else:
                current_waypoint += 1
                print(f"{mission_time:7.1f} | Waypoint {current_waypoint} reached!")
                
                # Initiate landing for final waypoint
                if current_waypoint == len(waypoints) - 1:
                    controller.initiate_landing()
                    print(f"{mission_time:7.1f} | Landing sequence initiated")
        
        # Status output (every 2 seconds)
        if int(mission_time * 10) % 20 == 0:
            pos_str = f"({rocket_state.position.x:4.1f},{rocket_state.position.y:4.1f})"
            vel_str = f"({rocket_state.velocity.vx:4.1f},{rocket_state.velocity.vy:4.1f},{rocket_state.velocity.vz:4.1f})"
            print(f"{mission_time:7.1f} | {rocket_state.position.z:6.1f} | {pos_str:10} | {vel_str:10} | "
                  f"{rocket_state.fuel_remaining:7.1f} | {current_waypoint+1}")
        
        mission_time += dt
        
        # Check for mission failure
        if rocket.is_crashed():
            print(f"{mission_time:7.1f} | ❌ Mission failed: Rocket crashed!")
            return False
            
        if rocket.is_out_of_fuel():
            print(f"{mission_time:7.1f} | ❌ Mission failed: Out of fuel!")
            return False
    
    # Mission summary
    print("\n" + "=" * 60)
    print("📊 MISSION SUMMARY")
    print("=" * 60)
    
    final_state = rocket.state
    success = (final_state.position.z < 1.0 and 
              abs(final_state.velocity.vz) < 1.0 and 
              not rocket.is_crashed())
    
    print(f"Mission Result: {'✅ SUCCESS' if success else '❌ FAILED'}")
    print(f"Total Time: {mission_time:.1f} seconds")
    print(f"Final Position: ({final_state.position.x:.2f}, {final_state.position.y:.2f}, {final_state.position.z:.2f})")
    print(f"Final Velocity: ({final_state.velocity.vx:.2f}, {final_state.velocity.vy:.2f}, {final_state.velocity.vz:.2f}) m/s")
    print(f"Fuel Consumed: {rocket.initial_fuel_mass - final_state.fuel_remaining:.1f} kg")
    print(f"Fuel Remaining: {final_state.fuel_remaining:.1f} kg")
    print(f"Waypoints Completed: {current_waypoint + 1}/{len(waypoints)}")
    
    if success:
        landing_accuracy = math.sqrt(final_state.position.x**2 + final_state.position.y**2)
        print(f"Landing Accuracy: {landing_accuracy:.2f} m from target")
        
        if landing_accuracy < 1.0:
            print("🎯 Precision landing achieved!")
        elif landing_accuracy < 3.0:
            print("✅ Accurate landing achieved!")
        else:
            print("⚠️  Landing within acceptable limits")
    
    print("\n🚀 VTOL Mission Complete!")
    return success


if __name__ == "__main__":
    run_mission_simulation()