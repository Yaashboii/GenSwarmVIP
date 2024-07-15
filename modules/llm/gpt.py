from openai import AsyncOpenAI
from modules.llm import api_base, key_manager, BaseLLM
from tenacity import (
    retry,
    stop_after_attempt,
    wait_random_exponential,
    stop_after_delay,
)


class GPT(BaseLLM):
    """
    Class for interacting with OpenAI's GPT model.

    This class provides methods specific to OpenAI's GPT model.

    Args:
        client (AsyncOpenAI): AsyncOpenAI client for interacting with the API.
    """

    def __init__(self, client: AsyncOpenAI = None, model: str = "gpt-4o", memorize: bool = False,
                 stream_output: bool = False) -> None:
        super().__init__(model, memorize, stream_output)
        self.key = key_manager.allocate_key()
        self._client = AsyncOpenAI(api_key=self.key, base_url=api_base) if client is None else client

    @retry(
        stop=(stop_after_attempt(5) | stop_after_delay(500)),
        wait=wait_random_exponential(multiplier=1, max=60),
    )
    async def _ask_with_retry(self, temperature: float) -> str:
        """
        Helper function to perform the actual call to the GPT model with retry logic.
        """
        try:
            response = await self._client.chat.completions.create(
                model=self._model,
                messages=self._memories,
                temperature=temperature,
                stream=self._stream_output,
            )

            if self._stream_output:
                collected_chunks = []
                collected_messages = []
                async for chunk in response:
                    collected_chunks.append(chunk)
                    choices = chunk.choices if hasattr(chunk, "choices") else []
                    if len(choices) > 0:
                        chunk_message = (
                            choices[0].delta if hasattr(choices[0], "delta") else {}
                        )
                        collected_messages.append(chunk_message)

                full_reply_content = "".join(
                    [
                        m.content if hasattr(m, "content") and m.content is not None else ""
                        for m in collected_messages
                    ]
                )
                self._response = full_reply_content
                return full_reply_content
            else:
                return response.choices[0].message.content

        except Exception as e:
            from modules.file.log_file import logger

            logger.log(f"Error in _ask_with_retry: {e}", level="error")
            raise  # Re-raise exception to trigge


if __name__ == '__main__':
    import asyncio

    gpt= GPT()
    response = asyncio.run(gpt.ask("Hello, who are you?"))
    print(response)
