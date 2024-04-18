from modules.framework.files.base_file import BaseFile
from modules.utils.logger import setup_logger, LoggerLevel


class _Logger:
    _instance = None

    def __new__(cls):
        if not cls._instance:
            cls._instance = super().__new__(cls)
            cls._logger = setup_logger(cls.__class__.__name__, LoggerLevel.DEBUG)
            cls._file = None
        return cls._instance

    def set_file(self, file: BaseFile):
        self._file = file

    @property
    def message(self):
        return self._file.message

    @message.setter
    def message(self, content: str):
        self._file.message = content

    def log(self, content: str, level: str = 'info'):
        """
        Formats a message based on the provided style and logs the content.

        :param content: The message content to be formatted and logged.
        :param level: The style to format the message. Supported styles: stage, action,
                      prompt, response, success, error, warning.
        """
        color_mapping = {
            'stage': '***\n# <span style="color: blue;">Current Stage: *{}*</span>\n',
            'action': '## <span style="color: purple;">Current Action: *{}*</span>\n',
            'prompt': '### <span style="color: grey ;">Prompt: </span>\n{}\n',
            'response': '### <span style="color: black;">Response: </span>\n{}\n',
            'success': '#### <span style="color: gold;">Success: {}</span>\n',
            'error': '#### <span style="color: red;">Error: </span>\n{}\n',
            'warning': '#### <span style="color: orange;">Warning: </span>\n{}\n',
            'info': '#### <span style="color: black;">info: </span>\n{}\n',
            'debug': '#### <span style="color: black;">debug: </span>\n{}\n',
        }

        # Verify level is supported
        if level not in color_mapping:
            self._logger.error(f"Level {level} is not supported")
        log_action = {
            'stage': self._logger.info,
            'action': self._logger.debug,
            'prompt': self._logger.debug,
            'response': self._logger.info,
            'success': self._logger.info,
            'error': self._logger.error,
            'warning': self._logger.warning,
            'info': self._logger.info,
            'debug': self._logger.debug,
        }.get(level, self._logger.info)

        log_action(content)

        self.message = color_mapping[level].format(content)

logger = _Logger()