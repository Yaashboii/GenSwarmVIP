import sys
import threading
import rospy
from code_llm.srv import GetTargetPositions


def get_target_positions():
    rospy.wait_for_service('/get_target_positions')
    try:
        get_target_positions = rospy.ServiceProxy('/get_target_positions', GetTargetPositions)
        response = get_target_positions()
        return {info.id: (info.position.x, info.position.y) for info in response.target_positions}
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return {}


def run_robot(robot_id, target_position):
    from functions import initialize_ros_node, run_loop
    initialize_ros_node(robot_id=robot_id, target_position=target_position)
    run_loop()


def run_multiple_robot(start_idx, end_idx):
    rospy.init_node(f'multi_robot_publisher_node{start_idx}_{end_idx}', anonymous=True)
    target_positions = get_target_positions()

    threads = []
    for i in range(start_idx, end_idx + 1):
        thread = threading.Thread(target=run_robot, args=(i, target_positions[i]))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    a = sys.argv
    # start_id = 1
    # end_id = 10
    start_id = int(sys.argv[1])
    end_id = int(sys.argv[2])
    run_multiple_robot(start_id, end_id)
