"""
Basic VTOL simulation example
Demonstrates takeoff, hover, and landing sequence
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from simulation.simulation import VTOLSimulation, SimulationConfig
from control.thrust_vector_controller import Position


def main():
    """Run basic VTOL simulation"""
    
    # Create simulation configuration
    config = SimulationConfig(
        dt=0.01,            # 10ms time steps
        max_time=120.0,     # 2 minute maximum
        real_time_factor=10.0,  # 10x speed
        log_interval=0.1    # Log every 100ms
    )
    
    # Create simulation
    sim = VTOLSimulation(config)
    
    # Set mission target (hover at 30m altitude)
    sim.set_mission_target(Position(x=0, y=0, z=30))
    
    # Progress callback
    def progress_callback(simulation):
        if int(simulation.time) % 10 == 0 and simulation.time % 10 < 0.1:
            state = simulation.rocket.state
            print(f"t={simulation.time:6.1f}s  Alt={state.position.z:6.1f}m  "
                  f"Vel={state.velocity.vz:6.2f}m/s  Fuel={state.fuel_remaining:5.1f}kg  "
                  f"Phase={simulation.mission_phase}")
    
    # Run simulation
    success = sim.run(callback=progress_callback)
    
    # Print summary
    summary = sim.get_summary()
    print("\n" + "="*60)
    print("SIMULATION SUMMARY")
    print("="*60)
    print(f"Mission Success: {'YES' if success else 'NO'}")
    print(f"Total Time: {summary['total_time']:.1f} seconds")
    print(f"Max Altitude: {summary['max_altitude']:.1f} meters")
    print(f"Final Position: ({summary['final_position'].x:.1f}, "
          f"{summary['final_position'].y:.1f}, {summary['final_position'].z:.1f})")
    print(f"Final Velocity: ({summary['final_velocity'].vx:.2f}, "
          f"{summary['final_velocity'].vy:.2f}, {summary['final_velocity'].vz:.2f}) m/s")
    print(f"Fuel Remaining: {summary['fuel_remaining']:.1f} kg")
    print(f"Data Points Logged: {summary['data_points']}")


if __name__ == "__main__":
    main()