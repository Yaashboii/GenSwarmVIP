# EXAMPLE SCRIPT FOR CONSENSUS IN THE ROBOTARIUM

import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.graph import *
from rps.utilities.controllers import *
import numpy as np

# Minimal consensus loop (conceptual)
N = 4
L = completeGL(N)  # complete graph Laplacian (utilities provided by the Robotarium package)
r = robotarium.Robotarium(number_of_robots=N, show_figure=True, sim_in_real_time=True)
_, uni_to_si_states = create_si_to_uni_mapping()

for t in range(200):
    x = r.get_poses()
    xi = uni_to_si_states(x)
    # consensus input: negative Laplacian times positions (simple)
    dxi = -xi.dot(L.T)
    # map and send (placeholder mapping/dynamics)
    r.step()

r.call_at_scripts_end()