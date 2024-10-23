import os
import cv2
import rospy
# from my_custom_srvs.srv import StartEnvironment, StartEnvironmentResponse
# from my_custom_srvs.srv import StopEnvironment, StopEnvironmentResponse

from robot_manager import Manager


class EnvironmentManager:
    def __init__(self, env, default_fps=30, max_speed=1.0):
        """
        初始化环境管理器，不再需要在构造函数中指定实验路径和ID。
        """
        self.env = env
        self.env.reset()
        self.experiment_path = None  # 动态设置实验路径
        self.exp_id = None  # 动态设置实验ID
        self.fps = default_fps  # 默认帧率
        self.manager = Manager(self.env, max_speed=max_speed)
        self.frames = []
        self.rendering_enabled = False
        self.running = False
        self.frame_dir = None

        # 注册 ROS 服务
        rospy.Service('/start_environment', StartEnvironment, self.handle_start_environment)
        rospy.Service('/stop_environment', StopEnvironment, self.handle_stop_environment)

    def handle_start_environment(self, req):
        """
        处理启动环境的请求，传入路径、最大速度、FPS等参数。
        """
        if not self.running:
            self.experiment_path = req.experiment_path  # 设置实验路径
            self.fps = req.fps  # 设置FPS
            self.manager.max_speed = req.max_speed  # 设置最大速度

            # 创建实验目录
            if not os.path.exists(self.experiment_path):
                os.makedirs(self.experiment_path)

            self.start_environment()
            return StartEnvironmentResponse(success=True, message="Environment started successfully.")
        return StartEnvironmentResponse(success=False, message="Environment is already running or failed to start.")

    def handle_stop_environment(self, req):
        """
        处理停止环境的请求，传入保存的文件名。
        """
        if self.running:
            self.stop_environment(req.file_name)
            return StopEnvironmentResponse(success=True, message="Environment stopped successfully.")
        return StopEnvironmentResponse(success=False, message="Environment is not running or failed to stop.")

    def start_environment(self):
        """
        启动环境并开始实验。
        """
        self.running = True
        self.env.reset()
        print(f"Environment started successfully with path: {self.experiment_path}, FPS: {self.fps}, Max speed: {self.manager.max_speed}")

    def reset_environment(self):
        """
        重置环境，恢复初始状态。
        """
        self.env.reset()
        self.manager.clear_velocity()
        self.frames.clear()
        print("Environment reset successfully.")

    def stop_environment(self, file_name):
        """
        停止实验，并保存帧数据。
        """
        self.running = False
        self.save_frames_as_animations(file_name)
        print(f"Environment stopped and saved as {file_name} successfully.")

    def save_frames_as_animations(self, file_name):
        """
        将保存的帧数据存储为动画文件（GIF 和 MP4）。
        """
        # 保存为 GIF
        gif_path = os.path.join(self.experiment_path, f"{file_name}.gif")
        imageio.mimsave(gif_path, self.frames, fps=self.fps)
        print(f"Saved animation as GIF at {gif_path}")

        # 保存为 MP4
        mp4_path = os.path.join(self.experiment_path, f"{file_name}.mp4")
        height, width, layers = self.frames[0].shape
        size = (width, height)
        out = cv2.VideoWriter(mp4_path, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, size)

        for frame in self.frames:
            out.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))

        out.release()
        print(f"Saved animation as MP4 at {mp4_path}")
        self.frames.clear()
