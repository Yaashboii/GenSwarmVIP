import pygame
import rospy

from modules.env.configurable_env import SimulationEnvironment
from modules.env.cross_env import CrossEnvironment
from modules.env.manager import Manager


def main():
    pygame.init()

    # env = SimulationEnvironment(1000, 1000, data_file='env_config/base.json')
    env = CrossEnvironment(1000, 1000, radius=450, robot_num=10, obstacle_num=10)
    screen = pygame.display.set_mode((env.width, env.height))
    clock = pygame.time.Clock()

    manager = Manager(env)
    manager.publish_observations()

    running = True
    rate = rospy.Rate(10)
    env.save_entities_to_file()
    draw_counter = 0
    draw_frequency = 1  # 每5帧绘图一次
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        dt = clock.tick(50) / 1000
        env.update(dt)
        if draw_counter % draw_frequency == 0:
            env.draw(screen)
        manager.publish_observations()
        # rospy.spin()
        draw_counter += 1

        rate.sleep()

    pygame.quit()
    env.save_entities_to_file()


if __name__ == "__main__":
    main()
