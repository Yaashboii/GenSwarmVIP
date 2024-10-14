import re


def parse_text(text: str, lang: str = "python", all_matches: bool = False) -> str | list[str]:
    pattern = rf"```{lang}.*?\s+(.*?)```"
    matches = re.findall(pattern, text, re.DOTALL)

    if not matches:
        # TODO: user-defined error
        error_message = f"Error: No '{lang}' code block found in the text."
        raise ValueError(error_message)

    if all_matches:
        return matches
    else:
        return matches[0]
