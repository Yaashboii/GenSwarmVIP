from modules.file import logger
from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree
from modules.framework.code_error import Feedback
from modules.framework.constraint import ConstraintPool
from modules.framework.parser import parse_text
from modules.prompt import (
    VIDEO_PROMPT_TEMPLATE,
    OUTPUT_TEMPLATE,
    TASK_DES,
)



class VideoCriticize(ActionNode):
    def __init__(self, next_text: str = "",
                 node_name: str = "",
                  ):

        super().__init__(next_text, node_name)

        self.frames: list
        self._function_pool = FunctionTree()
        self._constraint_pool = ConstraintPool()

    def _build_prompt(self):

        self.prompt = [VIDEO_PROMPT_TEMPLATE.format(task_des=TASK_DES,
                                                    command=self.context.command,
                                                    feedback="/n".join(self.context.feedbacks),
                                                    constraint=str(self._constraint_pool),
                                                    out_put=OUTPUT_TEMPLATE),
                       *map(lambda x: {"type": "image_url",
                                       "image_url": {"url": f'data:image/jpg;base64,{x}', "detail": "low"}},
                            self.frames),
                       ]
        pass

    def setup(self, frames):
        self.frames = frames

    async def _process_response(self, response: str) -> str | Feedback:
        response = parse_text(text=response, lang="json")
        result = eval(response)
        if result["result"].strip().lower() == "success":
            return result["feedback"]
        elif result["result"].strip().lower() == "fail":
            return Feedback(result["feedback"])
        else:
            logger.log(f"Invalid result: {result}", "error")
            raise Exception("Invalid result")


if __name__ == '__main__':
    import asyncio
    from modules.utils import root_manager

    path = "../../../workspace/2024-05-26_17-19-56"
    root_manager.update_root(path)
    function_analyser = VideoCriticize("analyze constraints")

    function_analyser.context.load_from_file(f"{path}/RunCodeAsync.pkl")
    asyncio.run(function_analyser.run())
    function_analyser.context.save_to_file(f"{path}/analyze_functions.pkl")
