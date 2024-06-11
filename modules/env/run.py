import threading
import rospy
from geometry_msgs.msg import Twist


def run_robot(robot_id):
    from functions import run_loop
    robot_id = sys.argv[1]

    run_loop()


def run_multiple_robot():
    rospy.init_node('multi_robot_publisher_node', anonymous=True)
    robot_start_index = rospy.get_param("robot_start_index", 0)
    robot_end_index = rospy.get_param("robot_end_index", 10)
    threads = []
    for i in range(robot_start_index, robot_end_index):
        thread = threading.Thread(target=run_robot, args=(i,))
        threads.append(thread)
        thread.start()

    for thread in threads:
        thread.join()



if __name__ == "__main__":
    run_multiple_robot()
