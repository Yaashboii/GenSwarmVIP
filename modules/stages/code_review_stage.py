import asyncio
from modules.prompt.code_review_stage_prompt import CODE_REVIEW_PROMPT_TEMPLATE, HIGH_LEVEL_FUNCTION_REVIEW
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.stages.stage import Stage, StageResult
from modules.actions import CriticizeFunctions, HighLevelFunctionReview
from collections import deque
from modules.utils import TestResult


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

        for function in self.context.function_pool.functions.values():
            print("function.name = ", function.name)
            print("function.satisfying_constraints = ", function.satisfying_constraints)

        max_length = max(len(constraints) for constraints in all_func_constraints)

        padded_constraints = [constraints + [None] * (max_length - len(constraints)) for constraints in
                              all_func_constraints]

        for i in range(max_length):
            tasks = []
            for j, function in enumerate(self.context.function_pool.functions.values()):
                constraint = padded_constraints[j][i]
                if constraint is None:
                    continue
                other_functions = [f.content for f in self._context.function_pool.functions.values() if
                                   f.name != function.name]

                prompt = CODE_REVIEW_PROMPT_TEMPLATE.format(
                    task_des=TASK_DES,
                    robot_api=ROBOT_API,
                    env_des=ENV_DES,
                    other_functions='\n\n'.join(other_functions),
                    functions=function.content,
                    constraints=self.context.constraint_pool.constraints[constraint].text
                )
                task = asyncio.create_task(self._criticize_function(prompt))

                tasks.append(task)
            await asyncio.gather(*tasks)

    async def _critic_function_high_level(self, prompt, function_name: str):
        self._action = HighLevelFunctionReview()
        await self._action.run(prompt=prompt, function_name=function_name)

    async def _critic_functions_high_level(self):
        function_layers = self.context.function_pool.function_layer
        for i, layer in enumerate(function_layers[1:]):
            tasks = []
            self._context.log.format_message(f"Layer: {i + 1}", "warning")

            for function in layer:
                prompt = HIGH_LEVEL_FUNCTION_REVIEW.format(
                    task_des=TASK_DES,
                    robot_api=ROBOT_API,
                    env_des=ENV_DES,
                    function_name=function.name,
                    other_functions='\n\n'.join(
                        ['\n'.join(f.import_list) + f.content
                         for f in self.context.function_pool.functions.values() if
                         f.name != function.name]),
                    function_content=function.content,
                )
                task = asyncio.create_task(self._critic_function_high_level(prompt=prompt, function_name=function.name))
                tasks.append(task)
            await asyncio.gather(*tasks)

    async def _run(self) -> StageResult:
        await self._critic_functions_high_level()
        return StageResult()


if __name__ == '__main__':
    from modules.utils import root_manager

    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    # pkl_path = f'{path}/design_stage.pkl'
    # pkl_path = f'{path}/analyze_functions_stage.kl'
    pkl_path = f'{path}/write_run_stage.pkl'
    # pkl_path = f'{path}/design_functions_stage.pkl'
    # pkl_path = f'{path}/write_run_stage.pkl'
    reviewer = ReviewStage()
    reviewer.context.load_from_file(pkl_path)

    root_manager.update_root(path, set_data_path=False)
    asyncio.run(reviewer.run())
    # # programmer.context.save_to_file(f'{path}/coding_stage.pkl')
    # # programmer.context.save_to_file(f'{path}/write_functions_stage.pkl')
    # # programmer.context.save_to_file(f'{path}/design_functions_stage.pkl')
    reviewer.context.save_to_file(f'{path}/review.pkl')
