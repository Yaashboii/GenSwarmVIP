import asyncio
from modules.prompt.code_review_stage_prompt import CODE_REVIEW_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.stages.stage import Stage, StageResult
from modules.actions import CriticizeFunctions
from collections import deque


# TODO: Implement the ReviewStage class
class ReviewStage(Stage):
    def __init__(self, action: CriticizeFunctions = None):
        super().__init__()
        self._action = action

    async def _criticize_function(self, prompt: str):
        self._action = CriticizeFunctions()
        await self._action.run(prompt=prompt)

    async def _criticize_functions(self):
        all_func_constraints = [function.satisfying_constraints for function in
                                self.context.function_pool.functions.values()]

        max_length = max(len(constraints) for constraints in all_func_constraints)

        padded_constraints = [constraints + [None] * (max_length - len(constraints)) for constraints in
                              all_func_constraints]

        for i in range(max_length):
            tasks = []
            for j, function in enumerate(self.context.function_pool.functions.values()):
                constraint = padded_constraints[j][i]
                if constraint is None:
                    continue

                prompt = CODE_REVIEW_PROMPT_TEMPLATE.format(
                    task_des=TASK_DES,
                    robot_api=ROBOT_API,
                    env_des=ENV_DES,
                    functions=function.content,
                    constraints=self.context.constraint_pool.constraints[constraint].text
                )
                task = asyncio.create_task(self._criticize_function(prompt))

                tasks.append(task)
            await asyncio.gather(*tasks)

    async def _run(self) -> StageResult:
        await self._criticize_functions()
        return StageResult()


if __name__ == '__main__':
    from modules.utils import root_manager

    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    pkl_path = f'{path}/write_functions_stage.pkl'
    reviewer = ReviewStage()
    reviewer.context.load_from_file(pkl_path)

    root_manager.update_root(path, set_data_path=False)
    asyncio.run(reviewer.run())
    reviewer.context.save_to_file(f'{path}/review_stage.pkl')
