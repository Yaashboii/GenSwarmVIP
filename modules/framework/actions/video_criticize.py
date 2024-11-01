from os import listdir

from click import prompt

from modules.file import logger
from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree
from modules.framework.code_error import Feedback
from modules.framework.constraint import ConstraintPool
from modules.framework.parser import parse_text
from modules.llm import GPT
from modules.prompt import (
    VIDEO_PROMPT_TEMPLATE,
    OUTPUT_TEMPLATE,
    TASK_DES,
)


class VideoCriticize(ActionNode):
    def __init__(self, skill_tree, next_text: str = "",
                 node_name: str = "",
                 ):
        self.__llm = GPT(memorize=True, model='VLM')

        super().__init__(next_text, node_name, self.__llm)

        self._frames: list
        self._skill_tree = skill_tree
        self._constraint_pool = ConstraintPool()

    def _build_prompt(self):
        self.setup()

        self.prompt = [VIDEO_PROMPT_TEMPLATE.format(task_des=TASK_DES,
                                                    instruction=self.context.command,
                                                    command=self.context.command,
                                                    feedback="/n".join(self.context.feedbacks),
                                                    constraint=str(self._constraint_pool),
                                                    out_put=OUTPUT_TEMPLATE),
                       *map(lambda x: {"type": "image_url",
                                       "image_url": {"url": f'data:image/jpg;base64,{x}', "detail": "low"}},
                            self._frames),
                       ]
        print(prompt)
        pass

    def setup(self):
        from modules.utils import root_manager, process_video, create_video_from_frames
        self.context.vlm = True
        video_path = f"{root_manager.workspace_root}/wo_vlm.mp4"
        self._frames = process_video(video_path, start_time=3, end_time=10, seconds_per_frame=0.5)
        create_video_from_frames(self._frames, output_path=f"{root_manager.data_root}/extra.mp4")

    async def _process_response(self, response: str) -> str | Feedback:
        response = parse_text(text=response, lang="json")
        result = eval(response)

        # Process success cas
        if result["result"].strip().lower() == "success":
            if self.context.args.human_feedback == 'True':
                if_feedback = input("If task is done? Press y/n: ")
                if if_feedback.lower() == "y":
                    logger.log("run code: success", "warning")
                    return "NONE"
                else:
                    if self.context.args.feedback == 'None':
                        logger.log("run code: fail", "warning")
                        return 'NONE'
                    feedback = input("Please provide feedback: ")
                    self.context.feedbacks.append(feedback)
                    return Feedback(feedback)
            else:
                return result["feedback"]

        # Process fail case
        elif result["result"].strip().lower() == "fail":
            return Feedback(result["feedback"])

        # Invalid result case
        else:
            logger.log(f"Invalid result: {result}", "error")
            raise Exception("Invalid result")

    # async def _run(self) -> str:
    #     sim = """```json
    #     {
    #     "result": "SUCCESS",
    #     "feedback": "..."
    #     }```
    #     """
    #     res = await self._process_response(response=sim)
    #     return res


if __name__ == '__main__':
    import asyncio
    from modules.utils import root_manager, process_video, create_video_from_frames

    path = "../../../workspace/encircling/2024-10-21_03-18-15"
    root_manager.update_root(path)
    function_analyser = VideoCriticize("analyze constraints")

    function_analyser.context.load_from_file(f"{path}/RunCodeAsync.pkl")
    asyncio.run(function_analyser.run())
    function_analyser.context.save_to_file(f"{path}/analyze_functions.pkl")
