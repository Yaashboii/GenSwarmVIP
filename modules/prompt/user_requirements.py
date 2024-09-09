tasks = {
    "bridging": """The robots need to evenly form a straight line bridge at the position where x=0 within the range of −2<y<2.
                 The number of robots is not fixed, so you can design a highly efficient target allocation algorithm based on robot id.
                 It is important to collaborate effectively and use high-efficiency algorithms to find your position while also avoiding competition for the same spot with others.
                 Design a safe and efficient algorithm to reach the target position as quickly as possible.""".strip(),

    "flocking": "Integrate into one flock by collaborating with all of robots within the map's range, "
                "adhering to cohesion by staying connected with all robots within the map's range, "
                "alignment by moving together with all the robots within the map's range,"
                "and separation by maintaining at least 0.5 meters between robots."
                "Additionally, avoid collisions with obstacles and boundaries."
                "Keep a distance of 0.3 meters from obstacles."
                "Keep a distance of 0.3 meters from boundaries.",

    "covering": "Evenly sample target positions across the entire map, then assign the corresponding position based on each robot's ID."
                "The allocation algorithm should be efficient and conflict-free."
                "The number of robots is not fixed, but it is crucial to ensure that all robots are evenly distributed on the map."
                "Robot should adjust their velocity in real-time to avoid collisions and move towards its target positions.",
    "circling": "You need to form a circle with other robots around position (0,0).radius=1"
                " Avoid collisions with other robots."
                "The number of robots is not fixed, so you can design a highly efficient target allocation algorithm based on robot id."
                "It is important to collaborate effectively and use high-efficiency algorithms to find your position while also avoiding competition for the same spot with others."
                "Design a safe and efficient algorithm to reach the target position as quickly as possible.",

    "crossing": "The robot needs to maintain a distance of at least 0.15 meters from other robots and obstacles while moving to the target position to avoid collisions. "
                "During movement, it should implement adjustment strategies to avoid congestion among multiple robots and improve overall efficiency.",
    'shaping': "The robots need to form a specific shape, with each robot assigned a unique point on that shape to move to while avoiding collisions during the movement."
               "The target assignment algorithm must ensure that the total distance for all robots to reach their targets is minimized, and that each robot is assigned a unique target point.",
    'transportation': "You need to collaborate with other robots to transport the object to its target location.",
    'encircling': "The robots need to surround the target prey by evenly distributing themselves along a circle with a radius of 0.5, centered on the prey. They will adjust their positions in real-time based on the movement of the prey to achieve coordinated encirclement."
                  "During this process, the robots must avoid collisions with each other and with obstacles, maintaining a distance of at least 0.3 meters whenever possible.",
    "exploration": "Robots need to collaboratively explore an unknown area, and based on the number of robots, each robot is assigned a set of unexplored sub-areas."
                   "The allocation algorithm must ensure the efficiency of all robots, avoiding repetition or omission."
                   "Each robot needs to explore its assigned area, and reaching within 0.2 meters of the target point is considered successful. Additionally, robots must avoid collisions with other robots during the exploration process.",
    "clustering": """Collaborate with all robots within the map's range and form multiple subgroups based on their initial coordinates.
    Each subgroup is formed within one quadrant of the map, ensuring that robots in the same quadrant belong to the same subgroup.
    Within each subgroup, adhere to cohesion by staying connected with all robots in the subgroup.
    Follow alignment by moving together with all the robots within the subgroup.
    Maintain separation by keeping at least 0.5 meters between different subgroups, while ensuring appropriate spacing between robots within each subgroup.
    Additionally, avoid collisions with obstacles and boundaries:
    Keep a distance of 0.3 meters from obstacles.
    Keep a distance of 0.3 meters from boundaries."""

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
