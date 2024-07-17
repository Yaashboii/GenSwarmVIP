import sys
import threading
import rospy


# from code_llm.srv import GetTargetPositions, GetCharPoints, GetCharPointsRequest


# def get_target_positions():
#     rospy.wait_for_service('/get_target_positions')
#     try:
#         get_target_positions = rospy.ServiceProxy('/get_target_positions', GetTargetPositions)
#         response = get_target_positions()
#         return {info.id: (info.position.x, info.position.y) for info in response.target_positions}
#     except rospy.ServiceException as e:
#         print(f"Service call failed: {e}")
#         return {}
#
#
# def get_contour_points(character):
#     rospy.wait_for_service('/get_char_points')
#     try:
#         get_char_points = rospy.ServiceProxy('/get_char_points', GetCharPoints)
#         request = GetCharPointsRequest(character=character)
#         response = get_char_points(request)
#         points = [(point.x, point.y) for point in response.points]
#         return points
#     except rospy.ServiceException as e:
#         print(f"Service call failed: {e}")
#         return []


def run_robot(robot_id):
    from functions import initialize_ros_node, run_loop
    print(f'run code success with id={robot_id}')
    initialize_ros_node(robot_id=robot_id)
    run_loop()


if __name__ == "__main__":
    import socket
    import re

    numbers = re.findall(r'\d+', socket.gethostname())

    numbers = list(map(int, numbers))
    if len(numbers) > 1:
        raise SystemExit(f"hostname:{socket.gethostname()},get numbers:{numbers}")
    run_robot(robot_id=numbers[0])
