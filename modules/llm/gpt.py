from openai import OpenAI

from modules.llm.api_keys import api_base, key_manager


class GPT():
    """
    Initialize the GPT class for interacting with OpenAI's GPT model.

    GPT provides basic methods for interacting with the model and parsing its 
    output.

    Args:
        key (str): OpenAI API key for authentication.
        model (str): Model to use (e.g., gpt-4 or gpt-3.5-turbo-16k).
        temperature (float): Controls the randomness of the model's output.
    """

    def __init__(self, model: str = 'gpt-3.5-turbo-1106', 
                 temperature: float = 0.7) -> None:
        self._model = model
        self._memories = []  # Current memories
        self._temperature = temperature
        key = key_manager.allocate_key()
        self._client = OpenAI(api_key=key, base_url=api_base)
        self._prompt = None
        self.response = None

    def generate_answer(self, prompt: str, temperature=0.7) -> str:
        """
        Generate an answer from the GPT model based on the given prompt.

        Args:
            prompt (str): The user's input prompt.
            temperature (float): Controls the randomness of the model's output.

        Returns:
            str: The generated answer from the GPT model.
        """
        self._memories = []
        self._memories.append({"role": "user", "content": prompt})
        self._prompt = prompt
        try:
            response = self._client.chat.completions.create(
                model=self._model,
                messages=self._memories,
                temperature=temperature,
                # response_format={"type": "json_object"},
            )

            content = response.choices[0].message.content
            self._response = content
            return content
        except Exception as e:
            raise ConnectionError(f"Error in generate_answer: {e}")