TASK_DES = """
There are some mobile ground robots that can control their own speed, acquire their own location, and sense the positions and speeds of other robots within their field of view.  
There is a control center that can gather global information about all the robots and is responsible for task allocation, assisting the robots in performing collaborative tasks.  
The control center only performs task allocation at the beginning, and thereafter the robots' autonomous movement is entirely dependent on themselves. The allocator needs to consider the initial state information of all robots and assign corresponding sub-goals to each robot based on the task objectives. The designed allocation algorithm must provide an optimal allocation strategy while avoiding conflicts between robots' goals. Moreover, since the task allocation by the control center runs only once, the robots must explore independently in a dynamic environment based on the assigned tasks. Therefore, the goals assigned to each robot should be adaptable to such changes in the environment.  
Not all tasks require allocation by the control center. Robots obtain assigned sub-tasks through APIs; if no corresponding API is provided, the task can be completed independently by each robot.  
Currently, multiple AI assistants are collaborating step-by-step to write code that runs on both the control center and the ground robots.  
You are one of these assistants, and you need to understand this context and carry out your work accordingly.
""".strip()
