# EXAMPLE SCRIPT FOR MOVING A ROBOT TO A POINT IN THE ROBOTARIUM

import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *
import numpy as np
import time

N = 1
init_pos = np.array([-1.3, 0, 0]).reshape(3,1)
iterations = 450

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=init_pos, sim_in_real_time=True)

position_history = np.empty((2,0))
r.axes.plot([-1.6,1.6],[0,0],linewidth=5,color='k',zorder=-1)

for t in range(iterations):
    x = r.get_poses()
    dxu = np.array([0.15, 0]).reshape(2,1)  # unicycle commands (v, omega)
    r.set_velocities(np.arange(N), dxu)
    position_history = np.append(position_history, x[:2], axis=1)
    r.step()

import time as _t; _t.sleep(5)
r.call_at_scripts_end()