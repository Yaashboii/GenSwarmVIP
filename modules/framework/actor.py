import ast
import logging

from modules.llm.gpt import GPT
from modules.prompt.code_prompt import code_example

class Actor():
    """
    The Actor class represents an actor that interacts with the GPT model
    to generate Python code based on user queries.
    """

    def __init__(self) -> None:
        """
        Initializes an Actor instance with a GPT model and a logger.
        """
        # Initialize the GPT model and logger for the Actor class
        self._gpt = GPT()
        self._logger = logging.getLogger("Actor")
        self._logger.setLevel(logging.INFO)

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
        response = self._gpt.generate_answer(new_prompt)
        if not self._is_valid_code(response):
            return ""

        if log:
            self._logger.debug("User Query: %s", query)
            self._logger.debug("Generated Code: %s", response)
            
        return response
    
    def modify_code(self, src: str, suggestion: str, log: bool = True):
        new_prompt = (f"Modify the code based on the suggestion. Code:\n{src}"
                      f"\n\n Suggestion: {suggestion}"
                      "\n\n Remove ```python ```")
        
        response = self._gpt.generate_answer(new_prompt)
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
            self._logger.error("Invalid Python code in the response: %s", 
                               text)
            return False
        
if __name__ == "__main__":
    logging.basicConfig(level=logging.CRITICAL)
    actor = Actor()
    query = "计算两数之差"
    actor.generate_code(query)
