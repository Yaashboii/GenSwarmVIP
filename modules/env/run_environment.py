import os
from os import listdir, makedirs

import pygame
import rospy
import numpy as np
import cv2

from modules.env.env.configurable_env import SimulationEnvironment
from modules.env.env.cross_env import CrossEnvironment
from modules.env.env.formation_env import FormationEnvironment
from modules.env.env.pursuit_env import PursuitEnvironment
from modules.env.env.explore_env import ExploreEnvironment
from modules.env.env.sheepdog_env import SheepdogEnvironment
from modules.env.env.cover_env import CoverEnvironment
from modules.env.env.collect_env import CollectEnvironment
from modules.env.env.move_env import MoveEnvironment
from modules.env.manager import Manager
from modules.utils import root_manager


def main():
    pygame.init()

    env = SimulationEnvironment(1000, 1000, data_file='env_config/base.json')
    # env = CrossEnvironment(1000, 1000, radius=450, robot_num=150, obstacle_num=30)
    # env = FormationEnvironment(1000, 1000, robot_num=150)
    # env = PursuitEnvironment(1000, 1000, robot_num=10, obstacle_num=10)
    # env = ExploreEnvironment(1000, 1000, robot_num=4)
    # env = SheepdogEnvironment(1000, 1000, sheep_num=30, dog_num=3)
    # env = CoverEnvironment(1000, 1000, robot_num=200)
    # env = CollectEnvironment(1000, 1000, num_1=10, num_2=5, robot_num=3)
    # env = MoveEnvironment(1000, 1000, robot_num=10, obstacle_num=100)
    screen = pygame.display.set_mode((env.width, env.height))
    clock = pygame.time.Clock()

    manager = Manager(env)
    manager.publish_observations()

    running = True
    rate = rospy.Rate(10)
    env.save_entities_to_file()
    draw_counter = 0
    draw_frequency = 1  # 每帧绘图一次

    data_root = f"/home/derrick/catkin_ws/src/code_llm/workspace/{rospy.get_param('path', 'test')}"
    count = len(listdir(f"{data_root}/data/frames/"))  # this is used to number the 'frames' folder
    frame_dir = f"{data_root}/data/frames/frame{count}"
    if not os.path.exists(frame_dir):
        makedirs(frame_dir)

    frame_files = []

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            dt = clock.tick(100) / 1000
            env.update(dt)
            if draw_counter % draw_frequency == 0:
                env.draw(screen)
                frame = pygame.surfarray.array3d(screen).astype(np.uint8)
                frame = np.rot90(frame, 3)
                frame = np.flip(frame, axis=1)
                frame_image_path = os.path.join(frame_dir, f'{draw_counter}.png')
                cv2.imwrite(frame_image_path, cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                frame_files.append(frame_image_path)

            manager.publish_observations()
            draw_counter += 1

            rate.sleep()
    finally:
        print("Shutting down")
        pygame.quit()
        env.save_entities_to_file()


if __name__ == "__main__":
    main()
