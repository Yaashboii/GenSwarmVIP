from const import ENV_CODE, WORKSPACE_ROOT
from modules.stages.stage import Stage, StageResult
from modules.actions import ActionResult, WriteCode
from modules.utils import read_file
from modules.framework.prompts import *
from modules.framework.workflow_context import FileStatus, FileInfo

class CodingStage(Stage):
    def __init__(self,  action: WriteCode=None):
        super().__init__()
        self._action = action

    def update(self, data: StageResult):
        self._last_stage_result = data

    def _run(self) -> StageResult:
        code_files = self._context.code_files
        if "core.py" not in code_files:
            prompt = WRITE_CORE_PROMPT_TEMPLATE.format(
                instruction=self._context.analysis,
                code=ENV_CODE)
            filename = "core.py"
        elif code_files["core.py"].status != FileStatus.TESTED_PASS:
            code = read_file(WORKSPACE_ROOT, "core.py")
            prompt = REWRITE_CORE_PROMPT_TEMPLATE.format(
                code=code,
                error_message=code_files["core.py"].message)
            filename = "core.py"
        elif "run.py" not in code_files:
            core_code = read_file(directory=WORKSPACE_ROOT, filename='core.py')
            prompt = WRITE_MAIN_PROMPT_TEMPLATE.format(
                user_requirements=self._context.analysis,
                core_code=core_code,
                env_code=ENV_CODE)
            filename = "run.py"
        else:
            test_code = read_file(WORKSPACE_ROOT, "test_run.py")
            prompt = REWRITE_MAIN_PROMPT_TEMPLATE.format(
                test_code=test_code,
                error_message=code_files["run.py"].message)
            filename = "run.py"
        code_files[filename] = FileInfo()
        self._action.run(prompt=prompt, filename=filename)
        return StageResult(keys=[])