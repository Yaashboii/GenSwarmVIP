from enum import Enum

from pydantic import BaseModel, Field

class FileStatus(Enum):
    NOT_TESTED = 1
    TESTED_PASS= 3

class RunResult(Enum):
    WITH_ERROR = 1
    WITHOUT_ERROR = 2

class FileInfo(BaseModel):
    message: str = Field(default='')
    status: FileStatus = Field(default=FileStatus.NOT_TESTED)
    version: int = Field(default=0)

class WorkflowContext():
    _instance = None
    user_command: str
    analysis: str
    class_diagram: str
    sequence_diagram: str
    code_files: dict[str, FileInfo] = {}

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

if __name__ == "__main__":
    context = WorkflowContext()
    context.user_command = "this is a test"
    tt = context.code_files
    tt["test"] = "dd"
    print(context.code_files)
