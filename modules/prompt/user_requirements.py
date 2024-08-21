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

    "herding": "Some robots play the role of shepherd dogs. There are many sheep in the environment, and you need to work together to herd them into a designated area."
               "You must be fast while ensuring the sheep do not escape the boundaries of the environment.",

    "circling": "You need to form a circle with other robots around position (0,0). Avoid collisions with other robots."
                "The number of robots is not fixed, so you can design a highly efficient target allocation algorithm based on robot id."
                "It is important to collaborate effectively and use high-efficiency algorithms to find your position while also avoiding competition for the same spot with others."
                "Design a safe and efficient algorithm to reach the target position as quickly as possible.",

    "crossing": "The robot needs to maintain a distance of at least 0.15 meters from other robots and obstacles while moving to the target position to avoid collisions. During movement, it should implement adjustment strategies to avoid congestion among multiple robots and improve overall efficiency.",


    'collect': """
You need to collaborate with other robots to move "red" objects to the x > 900 side and "yellow" objects to the x < 100 side. Here are the detailed steps and requirements:

1. **Before the Task Starts:**
    - Allocate the optimal list of objects to be transported for each robot ID based on the robot's position and the positions of all objects to be transported.
    - All objects should be allocated to robots before the transport process begins.
    - Each object should only be assigned to one robot.

2. **Transport Process:**
    - The robot first moves to within 10 meters of the object (including its radius), then picks up the object and moves it to the target area.
    - After moving the object to the target area, the robot puts down the object and moves to the next object's position.
    - This cycle continues until all objects are transported.

3. **During Movement:**
    - The robot must maintain a distance of at least 50 meters from other objects in the environment, except for the object it is currently or about to transport.
    - The robot must maintain a distance of at least 50 meters from obstacles.
    - The robot must maintain a distance of at least 50 meters from other robots.
    """,
    'move': """
    You need to collaborate with other robots to transport the object to its target location.
    """,


    'encircling': """
You play the role of a shepherd dog, and you need to herd the flock to the position (0, 0)
Therefore, you need to calculate the minimum radius of a circle centered at (0, 0) that covers all the sheep. 
Then, the robots should be evenly distributed around this circle which `radius= calculated radius+ 2*robot radius`.
Assign points on the circumference based on the principle of minimizing the distance each robot needs to move to reach its target position.
""",

    "exploration": "Robots need to collaborate to explore the entire map area. Divide the map into several regions equal to the number of robots."
                   "Each region will be assigned to a specific robot, which will explore within this region."
                   "Each robot needs to plan an appropriate path. This path must ensure that every 0.5*0.5 sub-grid within its assigned region is explored."
                   "The robots move along this path to fully explore their assigned region.",


    'formation_move': """
    Firstly,you should evenly distribute with other robots in space around a circle with a center at (50, 500) and a radius of 30. 
    After all robots are evenly distributed(within 10m of the target position), the robots should move towards the target position (900, 500),during the movement,robots should maintain circular formation with a radius of 30m.
    maintain a distance of 50 meters from obstacles.
    """

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
