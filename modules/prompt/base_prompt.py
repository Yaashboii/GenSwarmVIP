from modules.prompt.analyze_stage_prompt import ANALYZE_CONSTRAINT_PROMPT_TEMPLATE, CONSTRAIN_TEMPLATE, \
    ANALYZE_SKILL_PROMPT_TEMPLATE, FUNCTION_TEMPLATE
from modules.prompt.design_stage_prompt import DESIGN_GLOBAL_FUNCTION_PROMPT_TEMPLATE, \
    DESIGN_LOCAL_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.coding_stage_prompt import WRITE_GLOBAL_FUNCTION_PROMPT_TEMPLATE, \
    WRITE_LOCAL_FUNCTION_PROMPT_TEMPLATE
from modules.prompt.code_review_stage_prompt import GLOBAL_CODE_REVIEW_PROMPT_TEMPLATE, \
    LOCAL_CODE_REVIEW_PROMPT_TEMPLATE
from modules.prompt.coding_stage_prompt import WRITE_GLOBAL_RUN_PROMPT_TEMPLATE, WRITE_LOCAL_RUN_PROMPT_TEMPLATE, \
    GLOBAL_RUN_OUTPUT_TEMPLATE


class Prompt:

    def __init__(self, ):
        self.action_map: dict = {
            "AnalyzeConstraints":
                ANALYZE_CONSTRAINT_PROMPT_TEMPLATE,
            "AnalyzeSkills":
                ANALYZE_SKILL_PROMPT_TEMPLATE,
            "DesignFunction": {
                "global": DESIGN_GLOBAL_FUNCTION_PROMPT_TEMPLATE,
                "local": DESIGN_LOCAL_FUNCTION_PROMPT_TEMPLATE,
            },
            "WriteFunction": {
                "global": WRITE_GLOBAL_FUNCTION_PROMPT_TEMPLATE,
                "local": WRITE_LOCAL_FUNCTION_PROMPT_TEMPLATE
            },
            'CodeReview': {
                "global": GLOBAL_CODE_REVIEW_PROMPT_TEMPLATE,
                "local": LOCAL_CODE_REVIEW_PROMPT_TEMPLATE
            },
            'WriteRun': {
                "global": WRITE_GLOBAL_RUN_PROMPT_TEMPLATE,
                "local": WRITE_LOCAL_RUN_PROMPT_TEMPLATE
            }
        }

    def get_prompt(self, action: str, scope: str) -> str:
        if action not in self.action_map:
            return ""
        if action == "AnalyzeSkills" or action == "AnalyzeConstraints":
            return self.action_map[action]

        return self.action_map[action][scope]
