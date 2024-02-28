from openai import AsyncOpenAI

from modules.llm.api_keys import api_base, key_manager
from tenacity import retry, stop_after_attempt, wait_random_exponential


class GPT:
    """
    Initialize the GPT class for interacting with OpenAI's GPT model.

    GPT provides basic methods for interacting with the model and parsing its
    output.

    Args:
        key (str): OpenAI API key for authentication.
        model (str): Model to use (e.g., gpt-4-turbo-preview or gpt-3.5-turbo-0125).
        temperature (float): Controls the randomness of the model's output.
    """

    system_prompt = "You are a helpful assistant."

    def __init__(self, model: str = 'gpt-4-turbo-preview',
                 temperature: float = 0.7) -> None:
        self._model = model
        self._memories = []  # Current memories
        self._temperature = temperature
        self.key = key_manager.allocate_key()
        self._client = AsyncOpenAI(api_key=self.key, base_url=api_base)
        self._response: str

    def reset(self) -> None:
        self._memories = []
        self._memories.append({"role": "system", "content": self.system_prompt})

    async def ask(self, prompt: str, temperature=0.7) -> str:
        """
        Asynchronously generate an answer from the GPT model based on the given prompt.
        """
        self._memories.append({"role": "user", "content": prompt})
        return await self._ask_with_retry(temperature)

    @retry(stop=stop_after_attempt(5), wait=wait_random_exponential(multiplier=1, max=60))
    async def _ask_with_retry(self, temperature: float) -> str:
        """
        Helper function to perform the actual call to the GPT model with retry logic.
        """
        try:
            response = await self._client.chat.completions.create(  # 确保这个调用是异步的
                model=self._model,
                messages=self._memories,
                temperature=temperature
            )
            content = response.choices[0].message.content
            self._response = content
            return content
        except Exception as e:
            print(f"Error in _ask_with_retry: {e}")
            raise  # Re-raise exception to trigger retry
