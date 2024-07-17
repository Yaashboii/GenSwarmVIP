ENV_DES = """
Environment:
    bounds:{'x_min':-2.5, 'x_max': 2.5, 'y_min': -2.5, 'y_max': 2.5}
    # Robot should keep a distance of at least 0.3m from the boundaries

Robot:
    max_speed: 0.2m/s (constant)
    Control Method: Omnidirectional speed control
    Initial position: random position in the environment
    Initial speed: np.array([0, 0])
""".strip()
