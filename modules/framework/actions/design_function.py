from modules.framework.action import ActionNode
from modules.utils import parse_code, extract_function_definitions, extract_top_level_function_names
from modules.prompt.design_stage_prompt import DesignFunction_PROMPT_TEMPLATE
from modules.prompt.robot_api_prompt import ROBOT_API
from modules.prompt.env_description_prompt import ENV_DES

class DesignFunction(ActionNode):
    def __init__(self, next_text: str , node_name: str = ''):
        super(DesignFunction, self).__init__(next_text, node_name)
        self.prompt = DesignFunction_PROMPT_TEMPLATE.format(
            analysis=self._context.analysis.message,
            robot_api=ROBOT_API,
            env_des=ENV_DES
        )
     
    def _process_response(self, response: str) -> str:
        code = parse_code(text=response)
        function_list = extract_function_definitions(code)
        functions_name_and_content = []
        if not function_list:
            self._context.log.formssage(f"Design Function Failed: No function detected in the response", "error")
            raise Exception  # trigger retry
        for function in function_list:
            function_name = extract_top_level_function_names(code_str=function)
            if not function_name:
                self._context.log.format_message(f"Design Function Failed: No function detected in the response",
                                                 "error")
                raise Exception  # trigger retry

            functions_name_and_content.append({"name": function_name[0], "content": function})
        self._context.function_list = functions_name_and_content
        # self._context.log.format_message(f"{code}", "response")
        return str(functions_name_and_content)
