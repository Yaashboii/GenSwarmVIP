ENV_DES = """
Environment:
    Environment is composed of a 2D plane with obstacles and robots.
    The robots and obstacles in the space are circular, and the avoidance algorithm is the same for both.
    There are only static obstacles and other robots in the environment.
Robot:
    max_speed: 0.2m/s (constant)
    Control Method: Omnidirectional speed control(The output after velocity-weighted superposition of different objectives.)
    Control frequency: 100Hz (the robot's velocity should be updated at least every 0.01s)
    Initial position: random position in the environment
    Initial speed: np.array([0, 0])
    Min distance to other object: > self.radius +obj.radius + distance_threshold (Depending on the specific task, prioritize completing the task correctly before minimizing the collision probability.)
    position_resolution: 0.05m (The threshold for considering the robot as having reached a designated position is greater than position_resolution.)
""".strip()
