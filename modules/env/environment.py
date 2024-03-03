import re
import sys
import os
import cv2
import numpy as np
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse

from .robot import Robots, Leader


class Env:
    def __init__(
            self,
            size=(10, 10),
            n_robots=3,
            dt=0.01,
            if_leader=False,
            leader_speed=2.0,
            render_interval=10,

    ):
        rospy.init_node('env_node', anonymous=True)
        self._size = size
        self._dt = dt
        self._render_interval = render_interval
        self._leader_speed = leader_speed
        self._robots = Robots(n_robots, self._size)
        if if_leader:
            self._leader = Leader(initial_position=(0, 0))
            self._leader_publisher = rospy.Publisher('/leader/position', Float32MultiArray, queue_size=1)
        else:
            self._leader = None
        self._robots_initial_positions = self._robots.positions.copy()
        self._position_publisher = rospy.Publisher('/robots/position', Float32MultiArray, queue_size=1)
        self._reset_service = rospy.Service('/reset_environment', SetBool, self.reset_environment_callback)
        rospy.set_param('robots_num', n_robots)
        rospy.Subscriber('/robots/velocity', Float32MultiArray, self.velocity_callback, callback_args=self._robots)

        # flag to indicate if the test is running,if running, the robots will save their positions to history
        self._run_test = False
        # flag to indicate if the frames should be rendered
        self._render_frames = False
        self._fig, self._ax = plt.subplots()

        # counter for total run time
        self._run_time = 0

    @staticmethod
    def velocity_callback(data, robots):
        """
        velocity_callback is a callback function for the velocity topic.
        """
        print("Received velocity")
        robots.velocities = np.array(data.data).reshape(-1, 2)

    def reset_environment_callback(self, req):
        """
        reset_environment_callback is a callback function for the reset_environment service.
        It resets the environment to its initial state and returns a success message.
        must be called before and after running the script
        """
        self.reset()
        self._render_frames = req.data
        print("set render frames to", self._render_frames)

        response = SetBoolResponse()
        response.success = True
        response.message = "Environment reset successful!"
        return response

    def reset(self):
        """
        Reset the environment to its initial state.
        """
        self._run_test = not self._run_test
        if not self._run_test:
            self._robots.positions = self._robots_initial_positions.copy()
            self._robots.velocities = np.zeros_like(self._robots.velocities)
            self._robots.history = [self._robots.positions.copy()]
            data_path = rospy.get_param('data_path', '.')
            generate_video_from_frames(frames_folder=f'{data_path}/frames', video_path=f'{data_path}/video.mp4')
            print("Test stopped! Waiting for new test...")
            print("Environment reset successful!")
        else:
            print("Test started!")

    def step(self):
        if self._run_test:
            if self._leader:
                self._leader.move(self._leader_speed, self._dt)
                self._leader_publisher.publish(Float32MultiArray(data=self._leader.position))
            self._robots.move_robots(self._dt)
            self._position_publisher.publish(Float32MultiArray(data=self._robots.positions.flatten().tolist()))
            self._run_time += 1
            if self._run_time % self._render_interval == 0:
                self.render()
        elif plt.get_fignums():
            plt.close(self._fig)

    def render(self):
        if not plt.get_fignums():
            self._fig, self._ax = plt.subplots()
        self._ax.clear()

        traj_len, robot_num, _ = self._robots.history.shape
        for i in range(robot_num):
            show_len = min(60, traj_len)
            self._ax.plot(self._robots.history[-show_len:, i, 0],
                          self._robots.history[-show_len:, i, 1],
                          label=f"Robot {i} path")
            self._ax.plot(self._robots.history[-1, i, 0],
                          self._robots.history[-1, i, 1],
                          'o',
                          label=f"Robot {i} position")
        self._ax.plot(self._leader.position[0], self._leader.position[1],
                      marker='*', markersize=12, color='r',
                      linestyle='None', label="Leader position")
        self._ax.set_xlim(-self._size[0], self._size[0])
        self._ax.set_ylim(-self._size[1], self._size[1])
        if self._render_frames:
            plt.draw()
            plt.pause(0.001)  # This is necessary for the plot to update
        data_path = rospy.get_param('data_path', '.')
        frame_id = len(self._robots.history)
        plt.savefig(f'{data_path}/frames/{frame_id}.png')

    def run(self):
        print("Environment started!")
        rate = rospy.Rate(1 / self._dt)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
        print("Environment stopped!")


def generate_video_from_frames(frames_folder, video_path, fps=10):
    frame_files = sorted(os.listdir(frames_folder), key=lambda x: int(re.search(r'\d+', x).group()))
    frame_files = [os.path.join(frames_folder, file) for file in frame_files]

    frame = cv2.imread(frame_files[0])
    height, width, layers = frame.shape
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')

    video = cv2.VideoWriter(video_path, fourcc, fps, (width, height))

    for frame_file in frame_files:
        video.write(cv2.imread(frame_file))

    cv2.destroyAllWindows()
    video.release()
    print(f"Video generated: {video_path}")


if __name__ == "__main__":
    env = Env(if_leader=True, n_robots=100, size=(10, 10))
    env.run()
