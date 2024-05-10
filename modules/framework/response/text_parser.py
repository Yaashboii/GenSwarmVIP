import re


def parse_text(text: str, lang: str = "python") -> str:
    pattern = rf"```{lang}.*?\s+(.*?)```"
    match = re.search(pattern, text, re.DOTALL)

    if not match:
        # TODO: user-defined error
        error_message = f"Error: No '{lang}' code block found in the text."
        raise ValueError(error_message)

    code = match.group(1)
    return code
