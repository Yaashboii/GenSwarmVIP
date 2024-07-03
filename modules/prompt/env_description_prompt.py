ENV_DES = """
Environment:
    bounds:{'x_min':0, 'x_max': 1000, 'y_min': 0, 'y_max': 1000}
    min_distance_resolution: 10 m

Robot:
    max_speed: 1m/s (constant)
    Control Method: Omnidirectional speed control
    Initial position: random position in the environment
    Initial speed: np.array([0, 0])
""".strip()
