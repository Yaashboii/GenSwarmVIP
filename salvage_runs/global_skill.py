#from global_apis import get_all_robots_id,get_all_robots_initial_position
from rps.utilities.transformations import create_si_to_uni_dynamics
import numpy as np
import scipy.optimize as opt

def apply_barrier_certificates(poses, velocities, safety_radius=0.12, detection_radius=0.5, beta=14, epsilon=0.03):
    """Modify velocities with collision avoidance using barrier certificates"""
    num_robots = poses.shape[1]
    safe_velocities = np.copy(velocities)
    safety_radius_sq = (safety_radius + epsilon) ** 2
    detection_radius_sq = detection_radius ** 2

    for i in range(num_robots):
        for j in range(i + 1, num_robots):
            diff = poses[:2, j] - poses[:2, i]
            dist_sq = diff.T @ diff

            if dist_sq <= detection_radius_sq and dist_sq > 1e-6:
                h = dist_sq - safety_radius_sq
                diff_normalized = diff / np.sqrt(dist_sq)
                vel_diff = safe_velocities[:, j] - safe_velocities[:, i]
                h_dot = -2 * diff_normalized.T @ vel_diff

                if h_dot + beta * (h ** 3) < 0:
                    adjustment_magnitude = max(-(h_dot + beta * (h ** 3)) / (4 * dist_sq), 0)
                    adjustment = 2 * adjustment_magnitude * diff_normalized
                    safe_velocities[:, i] -= adjustment
                    safe_velocities[:, j] += adjustment

    return clip_velocities(safe_velocities, max_speed=0.12)


def clip_velocities(velocities, max_speed=0.12):
    """Clip velocities to maximum speed magnitude while preserving direction"""
    magnitudes = np.linalg.norm(velocities, axis=0)
    scaling_factors = np.where(magnitudes > max_speed, max_speed / magnitudes, 1.0)
    return velocities * scaling_factors


def generate_wander_velocities(num_robots, max_wander_speed=0.12):
    """Generate random single-integrator velocities for wandering behavior"""
    angles = np.random.uniform(0, 2*np.pi, num_robots)
    vx = max_wander_speed * np.cos(angles)
    vy = max_wander_speed * np.sin(angles)
    return np.vstack((vx, vy))


def main_control_loop(r, num_robots):
    """Main control loop for robotarium simulation"""
    poses = r.get_poses()
    dxi = generate_wander_velocities(num_robots, max_wander_speed=0.12)
    safe_dxi = apply_barrier_certificates(poses, dxi, safety_radius=0.12)
    si_to_uni_dyn = create_si_to_uni_dynamics()
    dxu = si_to_uni_dyn(safe_dxi, poses)
    r.set_velocities(np.arange(num_robots), dxu)


def create_si_to_uni_dynamics(linear_velocity_gain=1, angular_velocity_limit=np.pi):
    """Create mapping from single-integrator to unicycle dynamics"""
    def si_to_uni_dyn(dxi, poses):
        a = np.cos(poses[2, :])
        b = np.sin(poses[2, :])
        dxu = np.zeros((2, dxi.shape[1]))
        dxu[0, :] = linear_velocity_gain * (a * dxi[0, :] + b * dxi[1, :])
        dxu[1, :] = angular_velocity_limit * np.arctan2(-b * dxi[0, :] + a * dxi[1, :], dxu[0, :]) / (np.pi/2)
        return dxu
    return si_to_uni_dyn
