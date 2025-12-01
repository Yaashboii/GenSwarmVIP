"""
Sanitized salvage of LLM-generated Robotarium controller.
- Created from run log at: workspace/gpt-4/gather_top_right/robotarium/2025-12-01_09-57-20_923036/log.md
- Purpose: provide a runnable `global_skill.py` without modifying the original run folder.

Usage:
- Copy this file into the run folder or import it from your experiment code.
- Example run (assuming Robotarium and dependencies are available):

    import rps.robotarium as robotarium
    from rps.utilities.transformations import create_si_to_uni_dynamics
    from global_skill import main_control_loop

    NUM_ROBOTS = 5
    r = robotarium.Robotarium(number_of_robots=NUM_ROBOTS, show_figure=True, sim_in_real_time=True)
    for _ in range(1000):
        main_control_loop(r, NUM_ROBOTS)
        r.step()

Note: This file includes local implementations of the helper functions extracted from the LLM log.
"""

import numpy as np


def compute_si_velocities(current_poses, target=np.array([1.0, 1.0]), gain=1.0, position_resolution=0.05):
    """
    Computes single-integrator velocities (dx, dy) for multi-robot systems to converge to a target position.
    Returns a 2xN numpy array where columns correspond to robots.
    """
    positions = current_poses[:2, :]
    # direction from robot to target
    direction_vectors = target.reshape(2, 1) - positions
    distances = np.linalg.norm(direction_vectors, axis=0)

    velocities = gain * direction_vectors
    velocities[:, distances <= position_resolution] = 0.0
    return velocities


def clip_velocities(dxi, max_speed=0.12):
    """
    Scales columns of `dxi` so no column magnitude exceeds `max_speed`.
    dxi: 2xN numpy array
    """
    magnitudes = np.linalg.norm(dxi, axis=0)
    scaling = np.ones_like(magnitudes)
    exceed = magnitudes > max_speed
    if np.any(exceed):
        scaling[exceed] = max_speed / magnitudes[exceed]
    return dxi * scaling


def apply_barrier_certificates(si_velocities, poses, robot_radius=0.08, safety_radius=0.2, max_speed=0.12):
    """
    Lightweight pairwise velocity correction to reduce approach velocity when robots are closer than `safety_radius`.

    This is a pragmatic, real-time-friendly projection: for each robot pair that is
    too close and moving toward each other, remove the approach component equally
    from each robot's velocity.
    """
    num_robots = si_velocities.shape[1]
    positions = poses[:2, :]
    safe_velocities = np.copy(si_velocities)

    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            delta_p = positions[:, i] - positions[:, j]
            dist = np.linalg.norm(delta_p)
            if dist <= 0:
                continue
            if dist < safety_radius:
                direction = delta_p / dist
                dv = safe_velocities[:, i] - safe_velocities[:, j]
                dv_dot = float(np.dot(dv, direction))
                # If relative velocity has a component that reduces distance (<0), remove it
                if dv_dot < 0:
                    correction = dv_dot * direction
                    # apply half correction to each robot to keep symmetry
                    safe_velocities[:, i] -= 0.5 * correction
                    safe_velocities[:, j] += 0.5 * correction

    # final safety clipping to max_speed
    safe_velocities = clip_velocities(safe_velocities, max_speed=max_speed)
    return safe_velocities


def create_si_to_uni_dynamics(linear_velocity_gain=1.0, angular_velocity_limit=np.pi):
    """
    Fallback mapping from single-integrator velocities (2xN) and poses (3xN)
    to unicycle velocities (2xN): [v; w]. This mirrors the transformations found in the run workspace.
    """
    def si_to_uni(dxi, poses):
        assert isinstance(dxi, np.ndarray)
        assert isinstance(poses, np.ndarray)
        _, N = dxi.shape
        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])
        dxu = np.zeros((2, N))
        dxu[0, :] = linear_velocity_gain * (a * dxi[0, :] + b * dxi[1, :])
        # safe arctan-based angular component scaled to angular_velocity_limit
        # denominator can be near zero; use safe clip
        denom = dxu[0, :].copy()
        denom[denom == 0] = 1e-8
        dxu[1, :] = angular_velocity_limit * np.arctan2(-b * dxi[0, :] + a * dxi[1, :], denom) / (np.pi / 2)
        return dxu

    return si_to_uni


def main_control_loop(r, num_robots, target=np.array([1.0, 1.0]),
                      gain=1.0, position_resolution=0.05, max_speed=0.12,
                      robot_radius=0.08, safety_radius=0.2):
    """
    Single call per timestep controller for gathering robots to `target`.
    - Calls `r.get_poses()` exactly once per iteration.
    - Produces single-integrator velocities, applies barrier certificates,
      clips speeds, maps to unicycle via `create_si_to_uni_dynamics`, then
      calls `r.set_velocities()`.
    """
    # Get poses exactly once
    poses = r.get_poses()

    # Compute nominal single-integrator velocities toward target
    si_vels = compute_si_velocities(poses, target=target, gain=gain, position_resolution=position_resolution)

    # Apply barrier certificates to avoid collisions
    safe_si = apply_barrier_certificates(si_vels, poses, robot_radius=robot_radius, safety_radius=safety_radius, max_speed=max_speed)

    # Ensure speeds respect max_speed
    safe_si = clip_velocities(safe_si, max_speed=max_speed)

    # Map single-integrator to unicycle velocities
    si_to_uni = create_si_to_uni_dynamics()
    dxu = si_to_uni(safe_si, poses)

    # Send to robotarium
    # Robotarium expects 2xN array: [linear_velocity_row; angular_velocity_row]
    r.set_velocities(np.arange(num_robots), dxu)


# Optional: provide a small wrapper for quick local testing (commented out)
if __name__ == "__main__":
    try:
        from rps.robotarium_abc import RobotariumABC
        # If Robotarium is available in PYTHONPATH, user can run a quick sim here.
    except Exception:
        print("Robotarium not available in this environment. This file is a salvage artifact.")
