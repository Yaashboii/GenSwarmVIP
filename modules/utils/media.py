import os
import re
import cv2
import base64

import numpy as np

from modules.file.log_file import logger


def generate_video_from_frames(frames_folder, video_path, fps=15):
    logger.log(f"Generating video from frames in {frames_folder}...")
    try:
        frame_files = sorted(
            [file for file in os.listdir(frames_folder) if re.search(r"\d+", file)],
            key=lambda x: int(re.search(r"\d+", x).group()),
        )
    except Exception as e:
        logger.log(f"Error reading frames: {e}", level="error")
        return

    if not frame_files:
        logger.log("No frames found", level="error")
        return
    frame_files = [os.path.join(frames_folder, file) for file in frame_files]

    frame = cv2.imread(frame_files[0])
    height, width, layers = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    video = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for frame_file in frame_files:
        video.write(cv2.imread(frame_file))

    cv2.destroyAllWindows()
    video.release()
    logger.log(f"Video generated: {video_path}", level="info")


def process_video(video_path, seconds_per_frame=2, start_time=0, end_time=None):
    base64Frames = []
    video = cv2.VideoCapture(video_path)
    fps = video.get(cv2.CAP_PROP_FPS)
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))

    # Convert start_time and end_time to frames
    start_frame = int(start_time * fps)
    end_frame = total_frames if end_time is None else int(end_time * fps)

    # Ensure end_frame does not exceed total_frames
    end_frame = min(end_frame, total_frames)

    # Calculate the number of frames to skip
    frames_to_skip = int(fps * seconds_per_frame)
    curr_frame = start_frame

    # Loop through the video and extract frames at the specified sampling rate
    while curr_frame < end_frame:
        video.set(cv2.CAP_PROP_POS_FRAMES, curr_frame)
        success, frame = video.read()
        if not success:
            break
        _, buffer = cv2.imencode(".jpg", frame)
        base64Frames.append(base64.b64encode(buffer).decode("utf-8"))
        curr_frame += frames_to_skip

    video.release()
    logger.log(f"Extracted {len(base64Frames)} frames from {start_time}s to {end_time}s", level="info")
    return base64Frames


def create_video_from_frames(base64Frames, output_path, fps=30):
    frames = []
    for base64_frame in base64Frames:
        frame_data = base64.b64decode(base64_frame)
        np_frame = np.frombuffer(frame_data, np.uint8)
        frame = cv2.imdecode(np_frame, cv2.IMREAD_COLOR)
        frames.append(frame)
    if not frames:
        print("No frames to write to video")
        return
    height, width, layers = frames[0].shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Specify video codec
    video_writer = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
    for frame in frames:
        video_writer.write(frame)
    video_writer.release()
