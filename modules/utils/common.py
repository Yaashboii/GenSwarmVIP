import time

import rospy
from typing import Any
from std_srvs.srv import SetBool
from modules.file.log_file import logger


def call_reset_environment(data: bool):
    """

    Args:
        data (bool): Whether to render the environment
    """
    if not rospy.core.is_initialized():
        rospy.init_node('reset_environment_client', anonymous=True)

    rospy.wait_for_service('/reset_environment')
    try:
        reset_environment = rospy.ServiceProxy('/reset_environment', SetBool)
        resp = reset_environment(data)
        return resp.success, resp.message
    except rospy.ServiceException as e:
        logger.log(f"Service call failed: {e}", level='error')


def get_param(param_name):
    try:
        return rospy.get_param(param_name)
    except KeyError:
        print(f"Parameter not found: {param_name},retrying...")
        time.sleep(1)
        return get_param(param_name)
