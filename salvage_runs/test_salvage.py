#!/usr/bin/env python
"""
Test script for salvaged Robotarium global_skill.py
Runs a 1000-iteration simulation with 5 robots gathering at target [1.0, 1.0].

Usage:
    python ./test_salvage.py
"""

import sys
import importlib.util

# Load the salvage module using the full filename
salvage_path = 'gpt-4_gather_top_right_2025-12-01_09-57-20_923036_global_skill.py'
spec = importlib.util.spec_from_file_location("global_skill", salvage_path)
global_skill = importlib.util.module_from_spec(spec)
spec.loader.exec_module(global_skill)

main_control_loop = global_skill.main_control_loop

# Import Robotarium (assuming it's in PYTHONPATH or available via rps module)
try:
    import rps.robotarium as robotarium
except ImportError:
    print("Error: Could not import rps.robotarium. Ensure rps is installed and in PYTHONPATH.")
    exit(1)


def main():
    print("=" * 60)
    print("Robotarium Salvage Test: Gather 5 robots at [1.0, 1.0]")
    print("=" * 60)

    NUM_ROBOTS = 5
    NUM_ITERATIONS = 1000

    try:
        # Initialize Robotarium
        print(f"\nInitializing Robotarium with {NUM_ROBOTS} robots...")
        r = robotarium.Robotarium(
            number_of_robots=NUM_ROBOTS,
            show_figure=True,
            sim_in_real_time=True
        )
        print(f"✓ Robotarium initialized successfully")
        print(f"  Simulator time step: {r.time_step} s")
        print(f"  Target position: [1.0, 1.0]")
        print(f"  Running {NUM_ITERATIONS} iterations...\n")

        # Run control loop
        for iteration in range(NUM_ITERATIONS):
            # Call the main control loop (gathers robots to [1.0, 1.0])
            main_control_loop(r, NUM_ROBOTS)

            # Step the simulator
            r.step()

            # Periodic progress report
            if (iteration + 1) % 100 == 0:
                print(f"  Iteration {iteration + 1}/{NUM_ITERATIONS}")

        print(f"\n✓ Simulation completed successfully")

        # Print debug output (collision/boundary violations, etc.)
        print("\nCalling debug output...")
        r.call_at_scripts_end()

    except Exception as e:
        print(f"\n✗ Error during simulation: {e}")
        import traceback
        traceback.print_exc()
        return 1

    print("\n" + "=" * 60)
    print("Test completed. Check the Robotarium window for visualization.")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    exit(main())
