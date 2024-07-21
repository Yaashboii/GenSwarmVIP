from modules.deployment.entity import Landmark, Leader, Obstacle, PushableObject, Robot
from modules.deployment.gymnasium_env.base_env import GymnasiumEnvironmentBase
from modules.deployment.gymnasium_env.utils import *
from typing import Optional
from typing import TYPE_CHECKING, Any, Generic, SupportsFloat, TypeVar

ObsType = TypeVar("ObsType")
ActType = TypeVar("ActType")
RenderFrame = TypeVar("RenderFrame")


class GymnasiumClassificationEnvironment(GymnasiumEnvironmentBase):
    def __init__(self,
                 data_file: str = None,
                 radius: int = 1,
                 center: tuple = (0, 0)):
        super().__init__(data_file)
        self.radius = radius
        self.center = center

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self.entities = []
        self.init_entities()
        obs = self.get_observation("array")
        infos = self.get_observation("dict")
        return obs, infos

    def init_entities(self):
        entity_id = 0
        obstacle_size = 0.1 #self.data.get("entities").get("obstacle").get("size", 0.15)
        obstacle_shape = self.data.get("entities").get("obstacle").get("shape", "circle")

        for i in range(self.num_obstacles):

            position = sample_point(zone_center=[0, 0], zone_shape='rectangle', zone_size=[self.width, self.height],
                    robot_size=obstacle_size, robot_shape=obstacle_shape, min_distance=obstacle_size,
                    entities=self.entities)
            object = PushableObject(object_id=entity_id,
                                    initial_position=position,
                                    size=obstacle_size,
                                    color='blue')
            self.add_entity(object)
            entity_id += 1

        leader = Leader(leader_id=entity_id, initial_position=(0, 0), size=0.15)
        self.add_entity(leader)

    @staticmethod
    def find_farthest_points(points):
        points = np.array(points)
        distances = np.linalg.norm(points[:, np.newaxis] - points, axis=2)
        farthest_indices = np.argmax(distances, axis=1)
        return points[farthest_indices]

    @staticmethod
    def sample_points_on_circle(radius, center, num_points):
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        points = [(center[0] + radius * np.cos(angle),
                   center[1] + radius * np.sin(angle)) for angle in angles]
        return points

    @staticmethod
    def sample_points_inside_circle(radius, center, num_points, min_distance, max_attempts_per_point=10):
        def distance(p1, p2):
            return np.sqrt(np.sum((p1 - p2) ** 2, axis=1))

        points = []
        center = np.array(center)
        attempts = 0

        radius -= min_distance
        while len(points) < num_points and attempts < num_points * max_attempts_per_point:
            # Generate random points within the bounding square
            random_points = np.random.uniform(-radius, radius, size=(num_points, 2)) + center
            valid_mask = np.linalg.norm(random_points - center, axis=1) <= radius
            random_points = random_points[valid_mask]

            for point in random_points:
                if len(points) == 0 or np.all(distance(np.array(points), point) >= min_distance):
                    points.append(point)
                    if len(points) >= num_points:
                        break

            attempts += 1

        if len(points) < num_points:
            print(f"Warning: Could only place {len(points)} points out of {num_points} requested.")

        return np.array(points)


if __name__ == "__main__":

    import time
    import rospy

    from modules.deployment.utils.manager import Manager

    env = GymnasiumClassificationEnvironment("../../../config/env_config.json")

    obs, infos = env.reset()
    manager = Manager(env)
    manager.publish_observations(infos)
    rate = rospy.Rate(env.FPS)

    start_time = time.time()  # 记录起始时间
    frame_count = 0  # 初始化帧数计数器

    while True:
        # action = manager.robotID_velocity
        action = {}
        # manager.clear_velocity()
        obs, reward, termination, truncation, infos = env.step(action=action)
        env.render()
        manager.publish_observations(infos)
        rate.sleep()

        frame_count += 1  # 增加帧数计数器
        current_time = time.time()  # 获取当前时间
        elapsed_time = current_time - start_time  # 计算已过去的时间

        # 当达到1秒时，计算并打印FPS，然后重置计数器和时间
        if elapsed_time >= 1.0:
            fps = frame_count / elapsed_time
            print(f"FPS: {fps:.2f}")  # 打印FPS，保留两位小数
            frame_count = 0  # 重置帧数计数器
            start_time = current_time  # 重置起始时间戳
    print("Simulation completed successfully.")
