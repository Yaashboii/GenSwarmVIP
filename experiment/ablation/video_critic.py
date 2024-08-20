from modules.framework.handler import FeedbackHandler
from .base_alation import BaseAlation
from modules.framework.actions import VideoCriticize, Criticize
from modules.utils import process_video, create_video_from_frames


class VideoCriticAlation(BaseAlation):
    def __init__(self,
                 workspace_name: str,
                 exp_list: list):
        criticizer = VideoCriticize()
        super().__init__(criticizer, workspace_name, exp_list)

    def setup(self, directory: str):
        self.tester = VideoCriticize()
        hf_handler = FeedbackHandler()
        criticize = Criticize("feedback")
        hf_handler.next_action = criticize
        self.tester.error_handler = hf_handler
        self.tester.context.load_from_file(f"{directory}/WriteRun.pkl")
        frames = process_video(f"{directory}/animation.mp4",
                               end_time=30,
                               seconds_per_frame=2)
        create_video_from_frames(base64Frames=frames,
                                 output_path=f"{directory}/extra.mp4",
                                 fps=10)
        self.tester.setup(frames)

    async def run_single(self, directory: str):
        await self.tester.run()
        self.tester.context.save_to_file(f"{directory}/video_critic.pkl")
