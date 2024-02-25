import numpy as np
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32MultiArray
from robot import robots
from std_srvs.srv import SetBool, SetBoolResponse
from matplotlib.animation import FuncAnimation


class Env:
    def __init__(self, robot_list: list, size=(10, 10)):
        rospy.init_node('env_node', anonymous=True)
        self._size = size
        self._dt = 0.05
        self._robots = robot_list
        self._robots_initial_positions = [robot.position.copy() for robot in self._robots]
        self._render_frames = False
        self._fig, self._ax = plt.subplots()
        self._total_history = []
        self._position_publishers = []
        self._run_test = False
        self._reset_service = rospy.Service('/reset_environment', SetBool, self.reset_environment)
        for robot in self._robots:
            rospy.Subscriber(f'/robot_{robot.robot_id}/velocity', Float32MultiArray, self.velocity_callback,
                             callback_args=robot)
            self._position_publishers.append(
                rospy.Publisher(f'/robot_{robot.robot_id}/position', Float32MultiArray, queue_size=10))

    @staticmethod
    def velocity_callback(data, robot):
        print(f"Received velocity for robot {robot.robot_id}: {data.data}")
        robot.velocity = np.array(data.data)

    def reset_environment(self, req):
        self.reset()
        self._render_frames = req.data
        print("set render frames to", self._render_frames)

        response = SetBoolResponse()
        response.success = True
        response.message = "Environment reset successful!"
        return response

    def reset(self):
        self._run_test = not self._run_test
        round_history = []
        if not self._run_test:
            for i, robot in enumerate(self._robots):
                robot.position = self._robots_initial_positions[i].copy()
                robot.velocity = np.array([0, 0], dtype=float)
                round_history.append(robot.history)
                robot.history = [robot.position.copy()]
            self._total_history.append(round_history)
        print("Environment reset successful!")
        if self._run_test:
            print("Test started!")
        else:
            print("Test stopped! Waiting for new test...")

    def step(self):
        if self._run_test:
            for robot in self._robots:
                new_position = robot.position + robot.velocity * self._dt
                robot.position = np.clip(new_position, -np.array(self._size) / 2, np.array(self._size) / 2)
                robot.history.append(robot.position.copy())
                self._position_publishers[robot.robot_id].publish(Float32MultiArray(data=robot.position.tolist()))

        if self._render_frames:
            self.render()
        elif plt.get_fignums():
            plt.close(self._fig)

    def render(self):
        if not plt.get_fignums():
            self._fig, self._ax = plt.subplots()
            plt.show(block=False)
        self._ax.clear()
        for robot in self._robots:
            history = np.array(robot.history)
            self._ax.plot(history[:, 0], history[:, 1], label=f"Robot {robot.robot_id} path")
            self._ax.plot(history[-1, 0], history[-1, 1], 'o', label=f"Robot {robot.robot_id} current")
        half_x = int(self._size[0] / 2)
        half_y = int(self._size[1] / 2)
        self._ax.set_xlim(-half_x, half_x)
        self._ax.set_ylim(-half_y, half_y)
        # self._ax.legend(loc='upper right')
        plt.draw()
        plt.pause(0.001)  # This is necessary for the plot to update

    def save_animation(self, histories, file_name):
        print("Generating animation from robot history...")
        fig, ax = plt.subplots()

        histories = np.array(histories)
        frames = histories.shape[1]
        robots_num = histories.shape[0]

        def update_frame(frame_number):
            ax.clear()
            half_x = int(self._size[0] / 2)
            half_y = int(self._size[1] / 2)
            ax.set_xlim(-half_x, half_x)
            ax.set_ylim(-half_y, half_y)
            for i in range(robots_num):
                ax.plot(histories[i, :frame_number, 0], histories[i, :frame_number, 1],
                        label=f"Robot {i}")
                ax.plot(histories[i, frame_number, 0], histories[i, frame_number, 1], 'o')

        ani = FuncAnimation(fig, update_frame, frames=frames, interval=100)
        DATA_PATH = rospy.get_param('data_path', '.')
        ani.save(f'{DATA_PATH}/animation_{file_name}.gif', writer='pillow')
        print(f"Animation saved as animation_{file_name}.gif!")
        plt.close(fig)

    def run(self):
        print("Environment started!")
        rate = rospy.Rate(1 / self._dt)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()
        for i, history in enumerate(self._total_history):
            self.save_animation(history, str(i))
        print("Environment stopped!")


if __name__ == "__main__":
    env = Env(robots)
    env.run()
