import os
import re
import shutil
import time

import cv2
import rospy
from typing import Any
from enum import Enum
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

def generate_video_from_frames(frames_folder, video_path, fps=15):
    logger.log(f"Generating video from frames in {frames_folder}...")
    try:
        frame_files = sorted(
            [file for file in os.listdir(frames_folder) if re.search(r'\d+', file)],
            key=lambda x: int(re.search(r'\d+', x).group())
        )
    except Exception as e:
        logger.log(f"Error reading frames: {e}", level='error')
        return

    if not frame_files:
        logger.log("No frames found", level='error')
        return
    frame_files = [os.path.join(frames_folder, file) for file in frame_files]

    frame = cv2.imread(frame_files[0])
    height, width, layers = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    video = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for frame_file in frame_files:
        video.write(cv2.imread(frame_file))

    cv2.destroyAllWindows()
    video.release()
    logger.log(f"Video generated: {video_path}", level='info')
