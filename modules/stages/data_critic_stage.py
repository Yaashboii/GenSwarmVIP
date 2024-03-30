import asyncio
from modules.prompt.data_critic_stage_prompt import FILTER_CONSTRAINTS_TEMPLATE, OUTPUT_FORMAT
from modules.prompt.robot_api_prompt import DATA_API
from modules.prompt.task_description import TASK_DES
from modules.stages.stage import Stage, StageResult
from modules.actions import FilterConstraints


class DataCriticStage(Stage):
    def __init__(self, action: FilterConstraints = None):
        super().__init__()
        self._action = action

    async def _filter_constraints(self):
        prompt = FILTER_CONSTRAINTS_TEMPLATE.format(
            task_des=TASK_DES,
            data_api=DATA_API,
            output_format=OUTPUT_FORMAT,
            constraints='\n'.join([c.text for c in self._context.constraint_pool.constraints.values()]),
        )
        self._action = FilterConstraints()
        await self._action.run(prompt=prompt)

    async def _run(self) -> StageResult:
        await self._filter_constraints()
        return StageResult()


if __name__ == '__main__':
    from modules.utils import root_manager

    critic = DataCriticStage()
    path = '/home/derrick/catkin_ws/src/code_llm/workspace/test'
    pkl_path = f'{path}/analyze_constraints_stage.pkl'
    critic.context.load_from_file(pkl_path)
    root_manager.update_root(path, set_data_path=False)

    asyncio.run(critic.run())
    # # programmer.context.save_to_file(f'{path}/coding_stage.pkl')
    # # programmer.context.save_to_file(f'{path}/write_functions_stage.pkl')
    # # programmer.context.save_to_file(f'{path}/design_functions_stage.pkl')
    critic.context.save_to_file(f'{path}/filter_constraints.pkl')
