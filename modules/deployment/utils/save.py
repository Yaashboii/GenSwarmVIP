import os

import cv2
import imageio


def save_frames_as_animations(id, experiment_path, frames):
    # Save as GIF
    gif_path = os.path.join(f"../workspace/{experiment_path}", f'animation_{id}.gif')
    imageio.mimsave(gif_path, frames, fps=10)
    print(f"Saved animation for experiment  as GIF at {gif_path}")

    # Save as MP4
    mp4_path = os.path.join(f"../workspace/{experiment_path}", f'animation_{id}.mp4')
    height, width, layers = frames[0].shape
    size = (width, height)
    out = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*'mp4v'), 20, size)

    for frame in frames:
        out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

    out.release()
    print(f"Saved animation for experiment  as MP4 at {mp4_path}")

    frames.clear()
