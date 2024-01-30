import logging
import ast
import re
import json

from modules.prompt.code_prompt import code_example
from modules.roles.role import Role
from modules.actions import WriteCode, RephraseCommand, RunCode, DebugError
from modules.framework.message import Message


class Actor(Role):
    name: str = "Alex"
    profile: str = "Programmer"
    goal: str = "write elegant, readable, extensible, efficient code"
    constraints: str = (
        "the code should conform to standards like google-style and be modular and maintainable. "
        "Use same language as user requirement"
    )
    
    def __init__(self, **data) -> None:
        super().__init__(**data)
        self._init_actions([WriteCode, DebugError])
        self._watch([RephraseCommand, RunCode])

    async def _think(self, msg):
        if msg.cause_by == 'RephraseCommand':
            self.next_action = self.actions['WriteCode']
        elif msg.cause_by == 'RunCode':
                self.next_action = self.actions['DebugError']

    async def _act(self, msg) -> Message:
        rsp = await self.next_action.run(msg.content)
        msg = Message(content=rsp, role=self.profile,
                          cause_by=self.next_action, sent_from=self)
        return msg
    
    def generate_code(self, query: str, base_prompt: str = code_example, 
                      log: bool = True) -> str:
        """
        Generates Python code based on a user query.

        Args:
            query (str): The user's query.
            base_prompt (str): The base prompt for code generation.
            log (bool): Whether to log the user query and generated code.

        Returns:
            str: The generated Python code.
        """
        new_prompt = f"{base_prompt}\n# {query}"
        response = self._think(new_prompt)
        print(response)
        code = json.loads(response)["code"]
        print(code)
        if not self._is_valid_code(code):
            return ""

        if log:
            self._logger.debug("User Query: %s", query)
            self._logger.debug("Generated Code: %s", code)
            
        return code
    
    def modify_code(self, src: str, suggestion: str, log: bool = True):
        new_prompt = (f"Modify the code based on the suggestion. Code:\n{src}"
                      f"\n\n Suggestion: {suggestion}")
        
        response = self._think(new_prompt)

        pattern = r"```python(.*)```"
        match = re.search(pattern, response, re.DOTALL)
        response = match.group(1) if match else ""

        if not self._is_valid_code(response):
            return ""
        
        if log:
            self._logger.debug("User Query: %s", new_prompt)
            self._logger.debug("Generated Code: %s", response)

        return response
    
    def _is_valid_code(self, text: str) -> bool:
        """
        Checks if the provided text is a valid Python code.

        Args:
            text (str): The text to be checked.

        Returns:
            bool: True if the text is valid Python code, False otherwise.
        """
        try:
            ast.parse(text)
            return True
        except SyntaxError:
            self._logger.error("Invalid Python code in the response: %s", text)
            return False
        
if __name__ == "__main__":
    logging.basicConfig(level=logging.CRITICAL)
    actor = Actor()
    query = "计算两数之差"
    actor.generate_code(query)
