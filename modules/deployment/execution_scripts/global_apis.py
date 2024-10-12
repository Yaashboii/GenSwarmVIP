import rospy
from code_llm.msg import Observations
import numpy as np

initial_robot_positions = {}
initial_prey_positions = []
initial_unexplored_area = []
all_robots_id = []
init = False


def process_initial_observations(msg: Observations):
    global initial_robot_positions, initial_prey_positions, initial_unexplored_area, all_robots_id
    print("Processing initial observations...")
    initial_robot_positions.clear()
    initial_prey_positions.clear()
    initial_unexplored_area.clear()
    all_robots_id.clear()

    for obj in msg.observations:
        if obj.type == "Robot":
            position = np.array([obj.position.x, obj.position.y])
            initial_robot_positions[obj.id] = position
            all_robots_id.append(obj.id)
        elif obj.type == "Prey":
            position = np.array([obj.position.x, obj.position.y])
            initial_prey_positions.append(position)
        elif obj.type == "Landmark" and obj.color == "gray":
            initial_unexplored_area.append({
                "id": len(initial_unexplored_area),
                "position": np.array([obj.position.x, obj.position.y])
            })


def init_node():
    global init
    if init:
        return
    init = True
    print("Waiting for initial observations...")
    rospy.Subscriber("/observation", Observations, process_initial_observations)
    msg = rospy.wait_for_message("/observation", Observations)
    process_initial_observations(msg)
    print("Initial observations received.")


def get_all_robots_initial_position():
    init_node()
    return initial_robot_positions


def get_prey_initial_position():
    init_node()
    return initial_prey_positions


def get_initial_unexplored_area():
    init_node()
    return initial_unexplored_area


def get_all_robots_id():
    init_node()
    return all_robots_id
