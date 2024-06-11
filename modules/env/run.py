import sys
import threading
import rospy


def run_robot(robot_id):
    from functions import run_loop, initialize_ros_node
    initialize_ros_node(robot_id=robot_id)
    run_loop()


def run_multiple_robot(start_idx, end_idx):
    rospy.init_node(f'multi_robot_publisher_node{start_idx}_{end_idx}', anonymous=True)
    threads = []
    for i in range(start_idx, end_idx + 1):
        thread = threading.Thread(target=run_robot, args=(i,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()


if __name__ == "__main__":
    a = sys.argv
    start_id = int(sys.argv[1])
    end_id = int(sys.argv[2])
    run_multiple_robot(start_id, end_id)