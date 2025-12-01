import numpy as np
import rps.robotarium as robotarium
from global_skill import main_control_loop

# 1. Initialize the Robotarium
NUM_ROBOTS = 5  # Ensure this matches your task requirements
r = robotarium.Robotarium(number_of_robots=NUM_ROBOTS, show_figure=True, sim_in_real_time=True)

# 2. Run the simulation loop
print("Starting Simulation...")
max_iterations = 1000

try:
    for i in range(max_iterations):
        # Call the generated function from global_skill.py
        main_control_loop(r, NUM_ROBOTS)
        
        # Step the simulation
        r.step()
except Exception as e:
    print(f"Simulation error: {e}")

print("Simulation Finished.")