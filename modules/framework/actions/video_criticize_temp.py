from os import listdir

from openai import AsyncOpenAI

from modules.framework.action import ActionNode
from modules.framework.code.function_node import State, FunctionNode
from modules.framework.code.function_tree import FunctionTree
from modules.framework.constraint import ConstraintPool
from modules.framework.code.grammar_checker import GrammarChecker
from modules.framework.code_error import Bug, Bugs, Feedback

from modules.file.log_file import logger
from modules.framework.parser import parse_text
from modules.prompt import (
    VIDEO_PROMPT_TEMPLATE,
    OUTPUT_TEMPLATE,
    TASK_DES,
)
from modules.utils.media import process_video, create_video_from_frames


class VideoCriticize(ActionNode):
    def __init__(self, next_text: str = "", node_name: str = ""):

        super().__init__(next_text, node_name)

        self._frames: list
        self._function_pool = FunctionTree()
        self._constraint_pool = ConstraintPool()

    def _build_prompt(self):
        self.setup()

        self.prompt = [VIDEO_PROMPT_TEMPLATE.format(task_des=TASK_DES,
                                                    command=self.context.command,
                                                    feedback="/n".join(self.context.feedbacks),
                                                    constraint=str(self._constraint_pool),
                                                    out_put=OUTPUT_TEMPLATE),
                       *map(lambda x: {"type": "image_url",
                                       "image_url": {"url": f'data:image/jpg;base64,{x}', "detail": "low"}},
                            self._frames),
                       ]
        pass

    def setup(self):
        from modules.utils import root_manager

        number = len(listdir(f"{root_manager.data_root}/frames")) - 1
        video_path = f"{root_manager.data_root}/output{number}.mp4"
        self._frames = process_video(video_path, end_time=10, seconds_per_frame=1)
        create_video_from_frames(self._frames, output_path=f"{root_manager.data_root}/extra{number}.mp4")

    async def _process_response(self, response: str) -> str | Feedback:
        response = parse_text(text=response, lang="json")
        result = eval(response)
        if result["result"].strip().lower() == "success":
            # HumanFeedback
            if_feedback = input("If task is done? Press y/n")
            if if_feedback == "y":
                logger.log("run code:success", "warning")
                return "NONE"
            else:
                if self.context.args.feedback == 'None':
                    logger.log("run code:fail", "warning")
                    return 'NONE'
                feedback = input("Please provide feedback:")
                self.context.feedbacks.append(feedback)
                return Feedback(feedback)

        elif result["result"].strip().lower() == "fail":
            return Feedback(result["feedback"])
        else:
            logger.log(f"Invalid result: {result}", "error")
            raise Exception("Invalid result")

    async def _run(self) -> str:
        sim = """```json
        {
        "result": "SUCCESS",
        "feedback": "..."     
        }```
        """
        res = await self._process_response(response=sim)
        return res


if __name__ == '__main__':
    import asyncio
    from modules.utils import root_manager

    path = "../../../workspace/2024-05-26_17-19-56"
    root_manager.update_root(path)
    function_analyser = VideoCriticize("analyze constraints")

    function_analyser.context.load_from_file(f"{path}/RunCodeAsync.pkl")
    asyncio.run(function_analyser.run())
    function_analyser.context.save_to_file(f"{path}/analyze_functions.pkl")
