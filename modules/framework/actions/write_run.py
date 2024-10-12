from sympy.printing.dot import template

from modules.framework.action import ActionNode
from modules.framework.code import FunctionTree, State
from modules.framework.parser import SingleFunctionParser, parse_text
from modules.prompt import (
    GLOBAL_ROBOT_API,
    LOCAL_ROBOT_API,
    ALLOCATOR_TEMPLATE,
    GLOBAL_RUN_OUTPUT_TEMPLATE,
    ENV_DES,
    TASK_DES,
)
from modules.utils import root_manager


class WriteRun(ActionNode):
    def __init__(self, skill_tree: FunctionTree, next_text="", node_name=""):
        super().__init__(next_text, node_name)
        self._skill_tree = skill_tree

    def _build_prompt(self):
        functions = "\n\n".join(self._skill_tree.function_valid_content)
        robot_api = GLOBAL_ROBOT_API if self.context.scoop == "global" else (
                LOCAL_ROBOT_API + ALLOCATOR_TEMPLATE.format(template=self.context.global_skill_tree.output_template))
        self.desired_function_name = 'allocate_run' if self.context.scoop == 'global' else 'run_loop'
        self.prompt = self.prompt.format(
            task_des=TASK_DES,
            env_des=ENV_DES,
            robot_api=robot_api,
            functions=functions,
            template=GLOBAL_RUN_OUTPUT_TEMPLATE
        )

    async def _process_response(self, response: str) -> str:
        code = parse_text(text=response)
        parser = SingleFunctionParser()
        parser.parse_code(code)
        parser.check_function_name(self.desired_function_name)

        self._skill_tree.update_from_parser(parser.imports, parser.function_dict)
        self._skill_tree.save_code([self.desired_function_name])
        self._skill_tree[self.desired_function_name].state = State.WRITTEN

        if self._skill_tree.name == "global_skill":
            template = eval(parse_text(response, lang='json'))['value']

            self._skill_tree.output_template = (f"type: {template['type']}\n"
                                                f"description: {template['description']}\n")


if __name__ == '__main__':
    import asyncio
    from modules.framework.context import WorkflowContext

    context = WorkflowContext()
    path = "../../../workspace/test"
    context.load_from_file(f"{path}/reviewed_function.pkl")
    root_manager.update_root('../../../workspace/test')

    code_reviewer = WriteRun(context.global_skill_tree, )
    asyncio.run(code_reviewer.run())
    context.save_to_file("../../../workspace/test/write_run.pkl")
