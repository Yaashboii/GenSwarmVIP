ENV_DES = """
Environment:
    Environment is composed of a 2D plane with obstacles and robots.
    The environment is bounded.
    The environment is static.
Robot:
    max_speed: 0.2m/s (constant)
    Control Method: Omnidirectional speed control
    Control frequency: 100Hz (the robot's velocity should be updated at least every 0.01s)
    Initial position: random position in the environment
    Initial speed: np.array([0, 0])
    Perception range: 1.0m
    Min distance to other robots and obstacles: > self.radius +other.radius + distance_threshold (Depending on the specific task, prioritize completing the task correctly before minimizing the collision probability.)
    position_resolution: 0.05m (The threshold for considering the robot as having reached a designated position is greater than position_resolution.)
""".strip()
