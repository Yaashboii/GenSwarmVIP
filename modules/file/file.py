import os
from enum import Enum
from abc import ABC

from modules.file.base_file import BaseFile
from modules.utils import root_manager

from modules.file.log_file import logger

class FileStatus(Enum):
    NOT_WRITTEN = 0
    NOT_TESTED = 1
    TESTED_FAIL = 2
    TESTED_PASS = 3

class File(BaseFile):
    def __init__(self, name: str = '', message: str = '', root: str = ''):
        self.version = 0
        self._name = name
        self._root = root if root else root_manager.workspace_root
        self._status = FileStatus.NOT_WRITTEN
        self._message = message
    
    @property
    def message(self):
        if not self._message:
            try:
                self._message = self.read()
            except FileNotFoundError:
                self._message = ''
        return self._message
    
    @property
    def name(self):
        return self._name
    
    @message.setter
    def message(self, content: str):
        self._message = content
        if self._status == FileStatus.NOT_WRITTEN:
            self._status = FileStatus.NOT_TESTED
        self.write(content)

    @property
    def file_path(self):
        return os.path.join(self._root, self._name)

    def read(self):
        try:
            with open(self.file_path, 'r') as file:
                file_content = file.read()
            return file_content
        except FileNotFoundError:
            return f"File not found: {self.file_path}"

    def write(self, content, mode='w'):
        if not logger.is_file_exists():
            logger.set_file(File("log.md"))
        try:
            with open(self.file_path, mode) as file:
                file.write(content)
            operation = "written" if mode == 'w' else "appended"
            if operation == 'written':
                logger.log(f"File {operation}: {self.file_path}", level='info')
        except FileNotFoundError:
            logger.log(f"File not found: {self.file_path}", level='error')
        except Exception as e:
            logger.log(f"Error writing file: {e}", level='error')
    

    def copy(self, root, name=''):
        new_name = name if name else self._name
        new_file = File(root=root, name=new_name)
        new_file.message = self.message
        return new_file

