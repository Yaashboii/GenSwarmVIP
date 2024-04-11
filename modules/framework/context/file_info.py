from enum import Enum
from modules.utils import root_manager, read_file

class FileStatus(Enum):
    NOT_WRITTEN = 0
    NOT_TESTED = 1
    TESTED_FAIL = 2
    TESTED_PASS = 3

class FileInfo:
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        self.name = name
        self._message = message
        self.root = root
        self.status = FileStatus.NOT_WRITTEN
        self.version = 0

    @property
    def message(self):
        if not self._message:
            self.root = root_manager.workspace_root
            try:
                self._message = read_file(self.root, self.name)
            except FileNotFoundError:
                self._message = ''
        return self._message

    @message.setter
    def message(self, content: str):
        self._message = content
        if self.status == FileStatus.NOT_WRITTEN:
            self.status = FileStatus.NOT_TESTED
        from modules.utils import root_manager, write_file
        self.root = root_manager.workspace_root
        write_file(self.root, self.name, content)


class FileLog(FileInfo):
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        super().__init__(name, message, root)
        from modules.utils import setup_logger, LoggerLevel
        self._logger = setup_logger(self.__class__.__name__, LoggerLevel.DEBUG)

    @property
    def message(self):
        return super().message

    @message.setter
    def message(self, content: str):
        from modules.utils import root_manager
        self.root = root_manager.workspace_root
        from modules.utils import write_file
        write_file(self.root, self.name, content, mode='a')

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

logger = FileLog(name='log.md')