question = [
        "Gather these robots together",
        "Gather the robots to point (4,1)",
        "Gather the robots along the \"y = 3x\" trajectory",
        "Gather the robots along the \"y= 1/x\" trajectory",
        "Gather the robots along the \"y = x^2\" trajectory",
        "Robot No. 0 moves along y=x, and other robots gather around it",
        "Robot No. 0 moves along y=x^2, and other robots gather around it",
        "Robot 0 moves along the shape of the letter 'S', and the other robots gather around it at the same time",
        "Robot 0 moves along the shape of the letter 'U', and the other robots gather around it at the same time",
        "Cluster the robots along \"y = x\", moving from the lower left corner to the upper right corner"
        "Gather these robots together at the position of leader robot",
        "Gather the robots to point (2,3)",
        "Gather the robots along the y=x trajectory",
        'Move the robots to form a square formation',
        'First, move the robot to form a square formation. Then, move the robots to form a triangle formation.Finally gather these robots together',
        "Initially, gather all robots at the center of the environment, confirming their arrival before proceeding. Next, arrange the robots into a square formation with each side measuring exactly 1.0 meter, ensuring the formation's precision with right angles and equal sides. Once the square is confirmed, guide the robots to trace a circular path while maintaining the square formation. Constant monitoring is required to preserve the formation's integrity and the path's accuracy throughout the movement."
]
Analyze = """
1.There are some ground-moving robots in the room, and users will issue commands to direct their movement. 
2.You need to understand the user's commands and then analyze these commands. Consider what functions are needed to meet the user's requirements.
{env_des}
3.APIs: only these api can be used directly 
{api}
4.Common Sense: In the swarm task, there are three main algorithm to consider: 1. Accurately calculate the target position 2. control the robot to move to the target positon 3. evaluate the movement quality.
5.User requirements:{instruction}
6.Output the analysis in the following format:
User Requirement Description: Detailed description of the task the user hopes to automate.
Functional Requirements:
List the functions expected to be developed based on user needs and available resources.
For each function, briefly describe its purpose and expected outcome.
These functions should be decoupled from each other.
"""


DesignFunction = """
1.User requirements: {instruction}
In order to assist users in automating specific tasks, you need to design a series of decoupled Python functions based on user functional requirements and constraints.
- These functions should be decoupled from each other. 
- Each function can utilize the existing APIs.
- Each function should have strong reusability and should have a sufficient number of inputs and outputs.
- Each function does not need to provide the content of the function body; just giving a `pass` will suffice.

2.This is the existing environment API. 
{code}
{env_des}
Functions should try to reuse these APIs as much as possible and refer to the design of these APIs.

3.Common Sense: In the swarm task, there are three main algorithm to consider: 1. Accurately calculate the target position 2. control the robot to move to the target positon 3. evaluate the movement quality.

4.Please respond with the following format:
0)explanation: think step by step. How do you make the functions conform to the algorithm?
1)function list: write the function list in the following format, ```python\n<your response>```
Note:
- To avoid complexity, functions only need to provide the function name and docstring; you cannot write out the function body. You only need to design rather than write code.
- Every function should provide a detailed description of its functionality as well as any constraints that may need to be considered.
- stop the robot when it reaches the target position"""

WriteSeqDiagram_PROMPT_TEMPLATE = """
requirements document: {analysis}
function list: 
{robot_api}
{function_list}
{env_des}
Based on the user requirements document and current functions, you are required to design a sequence diagram.
Explanation: 
1. This sequence diagram is to call these existing functions to fulfill the user's requirements.
2. This sequence diagram can be directly translated into Python code, which is capable of continuously monitoring the environment and outputting control signals at a certain frequency.
3. The code generated from the sequence diagram should not loop infinitely; it should be able to exit the loop once the task is completed.
constrains: 
1. You need to use these existing functions to generate a call flow diagram to fulfill the user's requirements.
2. You can't define any new functions.
3. Ensure that the sequence diagram translates directly into executable code that can accomplish the task objectives.
4. stop the robot which reach the target position
Common Sense: In the swarm task, there are three main algorithm to consider: 1. Accurately calculate the target position 2. control the robot to move to the target positon 3. evaluate the movement quality.

You should respond to with:
0)explanation: think step by step. How do you make the diagram conform to the algorithm?
1)sequence diagram: use Mermaid's sequenceDiagram to write sequence diagram, ```mermaid\nsequenceDiagram\n <your response>```
You should only respond in the format as described below :
0)explanation:
1)sequence diagram:
"""

WRITE_FUNCTION_PROMPT_TEMPLATE = """
You need to complete this function, making maximum reuse of existing functions.
requirements document: {instruction}
Environment APIs:
{env_api}
{other_functions}

The functions you generate need to comply with the following constraints.
constrains: 
1. All environment APIs are readily accessible and can be invoked directly using their function names.
2. COMPLETE CODE: Your code will be part of the entire project, so please implement complete, reliable, reusable code snippets.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
5. Before using a external variable/module, make sure you import it first. But Environment APIs are readily accessible and can be invoked directly using their function names.
6. YOU MUST FOLLOW "Data structures and interface definitions". DONT CHANGE ANY DESIGN.
7. You only need to make sure single function has correct input and output. don't need to consider complex conditions.
8. You can only complete this one function; you cannot generate other Helper functions. If necessary, you can define functions within this function.

Please finish the following function:
{function}

"""

WRITE_RUN_PROMPT_TEMPLATE = """
requirements document: {instruction}
you need to translate the sequence diagram into Python code.
Sequence Diagram:
{sequence_diagram}
{env_des}
constrains: 
1. You can't define any new functions.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. 
3. All functions that appear in the sequence diagram do not need to be imported. However, if you need to use external libraries, you must import them correctly.
4. Ensure the generated code contains a 'if __name__ == '__main__':' statement and can be executed directly
You should respond to with:
0)explanation: think step by step. How to translate the sequence diagram into Python code.
2)python code: Translate the sequence diagram into corresponding Python code. ```python\n <your response>```
You should only respond in the format as described below :
0)explanation:
1)python code:
"""
