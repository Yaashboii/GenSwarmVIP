tasks = {
    "bridging": """The robots need to evenly form a straight line bridge at the position where x=0 within the range of −2<y<2.""".strip(),

    "flocking": "Integrate into a flock by collaborating with all robots within the map, ensuring cohesion by staying connected, alignment by moving together, and separation by keeping at least 0.5 meters apart.",

    "covering": "Evenly sample target positions across the entire map, then assign the corresponding position based on each robot's ID.",

    "circling": "You need to evenly distribute yourself and other robots on a circle with a radius of 1 centered at (0,0).",

    "crossing": "The robot needs to maintain a distance of at least 0.15 meters from other robots and obstacles while moving to the target position to avoid collisions. ",

    'shaping': "The robots need to form a specific shape, with each robot assigned a unique point on that shape to move to while avoiding collisions during the movement.",

    'encircling': "The robots need to surround the target prey by evenly distributing themselves along a circle with a radius of 1, centered on the prey, and adjust their positions in real-time based on the prey's movement to achieve coordinated encirclement.",

    "exploration": "The robots need to collaboratively explore the entire unknown area. For an unknown region, reaching within 0.5 meters of its center point is considered as having explored it.",

    "clustering": "The robots need to select an appropriate position to move to based on the quadrant of their initial position and gather with other robots while maintaining a distance of at least 0.5 meters from each other to avoid collisions.",
    "pursuing": "The robots need to collaborate in pursuing the target."

}


def get_user_commands(task_name: str | list = None) -> list[str]:
    """
    Description: Get the user commands to be implemented.
    Args:
        task_name: str|list, the name of the task to be implemented (default is None).
        options are ["flocking", "covering", "exploration", "pursuit", "crossing", "shaping", None]
        When task_name is None, return all the user commands.
    Returns:
        list[str]: The list of user commands to be implemented.
    """

    if task_name is None:
        return list(tasks.values())
    elif isinstance(task_name, str):
        return [tasks[task_name]]
    elif isinstance(task_name, list):
        return [tasks[task] for task in task_name]


def get_commands_name() -> list[str]:
    """
    Description: Get the names of the user commands to be implemented.
    Returns:
        list[str]: The list of names of user commands to be implemented.
    """
    return list(tasks.keys())
