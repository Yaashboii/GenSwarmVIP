from modules.stages.stage import Stage, StageResult
from modules.actions import WriteDesign
from const import ENV_CODE


PROMPT_TEMPLATE = """
Based on existing code and user requirements,you need to conceptualize how to design an algorithm to independently achieve the objective, output the corresponding class diagram, and the call graph of the algorithm.
Our project is divided into three files in total. Among them, `env.py` represents the pre-defined simulation environment conditions. 
`core.py` constitutes the core algorithms that fulfill user requirements.
`run.py` serves as an interface to the environment, directly executing the algorithms, and is the file where the user requirements are ultimately realized.
Now, you need to design a class diagram for this system, along with a diagram depicting the calling relationships.
requirements:
{instruction}
current code(env.py):
{code}

Use Mermaid's sequenceDiagram. and classDiagram.

Return ```mermaid  ``` with NO other texts,
"""


class DesignStage(Stage):
    def __init__(self,  action: WriteDesign):
        super().__init__()
        self._action = action

    def _run(self) -> StageResult:
        prompt = PROMPT_TEMPLATE.format(instruction=self._context.analysis, code=ENV_CODE)
        self._context.class_diagram = self._action.run(prompt=prompt)
        return StageResult(keys=[])