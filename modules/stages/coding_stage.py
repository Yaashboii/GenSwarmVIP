import asyncio

from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES
from modules.prompt.task_description import TASK_DES
from modules.stages.stage import Stage, StageResult
from modules.actions import WriteCode, DesignFunction
from modules.utils import CodeMode, extract_top_level_function_names
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE, WRITE_RUN_PROMPT_TEMPLATE
from modules.framework.workflow_context import FileStatus, WorkflowContext
from modules.utils import combine_unique_imports


class CodingStage(Stage):
    def __init__(self, action: WriteCode = None):
        super().__init__()
        self._action = action

    async def _design_function(self, prompt: str):
        self._action = DesignFunction()
        await self._action.run(prompt=prompt)

    async def _design_functions(self):
        tasks = []
        for i, function in self._context.function_pool.functions.items():
            constraint_text = ''
            for constraint in function.satisfying_constraints:
                if constraint not in self._context.constraint_pool.constraints:
                    print(f"Constraint {constraint} is not in the constraint pool")
                    raise SystemExit
                constraint_text += self._context.constraint_pool.constraints[constraint].text + '\n'

            function_list = [f.text for f in self._context.function_pool.functions.values() if f.name != function.name]
            prompt = DesignFunction_PROMPT_TEMPLATE.format(
                task_des=TASK_DES,
                robot_api=ROBOT_API,
                env_des=ENV_DES,
                function_name=function.name,
                function_des=function.description,
                constraints=constraint_text,
                other_functions='\n'.join(function_list)

            )
            task = asyncio.create_task(self._design_function(prompt))
            tasks.append(task)
        await asyncio.gather(*tasks)

    async def _write_functions(self):
        self._prompt = WRITE_FUNCTION_PROMPT_TEMPLATE.format(
            task_des=TASK_DES,
            robot_api=ROBOT_API,
            env_des=ENV_DES,
            req_constraints=self._context.analysis.message,
            existing_functions=self._context.function_pool.message
        )

        self._action = WriteCode()
        res = await self._action.run(prompt=self._prompt)
        if res == 'No Need':
            print('No need to write new function')
            return
        else:
            self._context.function_pool.add_functions(res)
            await self._write_functions()

    async def _run(self) -> StageResult:
        await self._design_functions()
        return StageResult()


if __name__ == '__main__':
    from modules.utils import root_manager
    import os
    import pickle

    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    pkl_path = f'{path}/analysis_stage.pkl'
    programmer = CodingStage(WriteCode())
    programmer.context.load_from_file(pkl_path)

    root_manager.update_root(path, set_data_path=False)
    if os.path.exists(path + '/functions.py'):
        os.remove(path + '/functions.py')
    asyncio.run(programmer.run())
