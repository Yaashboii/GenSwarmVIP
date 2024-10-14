import os

from experiment.ablation import VideoCriticAlation, HumanFeedbackAlation
from experiment.ablation.utils import extra_exp


async def main(exp_type: str, workspace_name: str, exp_list: list):
    if exp_type == "video_critic":
        tester = VideoCriticAlation(workspace_name, exp_list)
    elif exp_type == "human_feedback":
        tester = HumanFeedbackAlation(workspace_name, exp_list)

    await tester.run_exp()


if __name__ == '__main__':
    import asyncio

    exp_type = "video_critic"
    # exp_type = "human_feedback"
    exp_list = sorted(extra_exp("../../workspace/parallel/cross"))
    # exp_list = sorted("../../workspace/layer/cross")

    workspace_name = f'../../workspace/ablation/layer/{exp_type}'

    asyncio.run(main(exp_type=exp_type, workspace_name=workspace_name, exp_list=exp_list))
