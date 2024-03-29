import asyncio

from modules.framework.action import ActionNode
from modules.utils import parse_code, combine_unique_imports
from modules.prompt.coding_stage_prompt import WRITE_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES

class WriteFunctions(ActionNode):
    def __init__(self, next_text: str , node_name: str = ''):
        super().__init__(next_text, node_name)

        self.prompt = ''
                      
        self._filename = "functions.py"
  

    def _process_response(self, response: str) -> str:
        code = parse_code(text=response)
        if self._filename not in self._context.code_files:
            self._context.log.format_message(f"Write Code Failed: No filename found in context", "error")
            raise SystemExit

        return code