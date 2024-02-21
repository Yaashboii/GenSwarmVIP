WRITE_CORE_PROMPT_TEMPLATE = """
Based on existing code and user requirements, write a separate algorithm file.
requirements:
{instruction}
current code(env.py):
{code}
Write code with triple quoto, based on the following attentions and context.
1. Only One file: do your best to implement THIS ONLY ONE FILE.
2. COMPLETE CODE: Your code will be part of the entire project, so please implement complete, reliable, reusable code snippets.
3. Set default value: If there is any setting, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE. AVOID circular import.
4. CAREFULLY CHECK THAT YOU DONT MISS ANY NECESSARY CLASS/FUNCTION IN THIS FILE.
5. Before using a external variable/module, make sure you import it first.
6. Write out EVERY CODE DETAIL, DON'T LEAVE TODO.
8. Place the algorithm code in a separate file, and you must import the existing 'env' instance by 'from env import env'.
Return ```python your_code_here ``` with NO other texts,
your code:
"""

REWRITE_CORE_PROMPT_TEMPLATE = """
This is the source file you previously wrote, but there are error messages as follows. 
code:
```python
{code}
```
Error message:
{error_message}

The source file is at the same directory level as the environment file (filename=env.py).
Use 'from env import env' to import the environment file.


You need to rewrite the entire source file to correct the errors. 
Note: output the complete file.

you should correctly import the necessary classes based on these file locations!

Return ```python your_code_here ``` with NO other texts,
your code:
"""

WRITE_MAIN_PROMPT_TEMPLATE = """
This project has passed the acceptance tests, and now we need to write a core.py for the project. 
The purpose of this core.py is to be executable directly, fulfilling the initial user requirements without requiring any testing. 
I will provide you with some code snippets from the acceptance tests and all the source code references. 
Your task is to generate a correct core.py to the best of your ability.

user_requirements:{user_requirements}
codes:
core.py{core_code}
env.py{env_code}
All of these codes are in the same directory and can be directly called using 'from filename import ....'
You only need to write a very simple runtime code; creating any new algorithms is not allowed. You must invoke other already implemented algorithms. You can refer to the code in the acceptance tests for guidance.

Return ```python your_code_here ``` with NO other texts,
your code:
"""

REWRITE_MAIN_PROMPT_TEMPLATE = """
This is the source file you previously wrote, but there are error messages as follows. 
code:
```python
{code}
```
Error message:
{error_message}

codes:
    core.py
    ```
    {core_code}
    ```
    ----
    environment APIs:
    ```
    {env_code}
    ```
The test file is at the same directory level as the source file (filename=core.py).
Use 'from core import ...' and 'from env import env' to import the source file.


You need to rewrite the entire test file to correct the errors. 
Note: output the complete file.


you should correctly import the necessary classes based on these file locations!
Return ```python your_code_here ``` with NO other texts,
your code:
"""

WRITE_TEST_PROMPT_TEMPLATE = """
NOTICE
1. Role: You are a QA engineer; the main goal is to design, develop, and execute PEP8 compliant, well-structured, maintainable test cases and scripts for Python 3.9. Your focus should be on ensuring the product quality of the entire project through systematic testing.
2. Requirement: Based on the context, please develop complete, robust, and reusable unit test cases.
3. Attention1: If there are any settings in your tests, ALWAYS SET A DEFAULT VALUE, ALWAYS USE STRONG TYPE AND EXPLICIT VARIABLE.
4. Think before writing: What should be tested and validated in this document? What edge cases could exist? What might fail?
5. CAREFULLY CHECK THAT YOU DON'T MISS ANY NECESSARY TEST CASES/SCRIPTS IN THIS FILE.
-----
## Given the following code, please write appropriate test cases using Python's unittest framework to verify the correctness and robustness of this code:
```python
{code_to_test}
```
The test file is at the same directory level as the source file (filename=core.py).Use from core import ... and from env import env to import the source file.

you should correctly import the necessary classes based on these file locations!
Return ```python your_code_here ``` with NO other texts,
your code:
"""


REWRITE_TEST_PROMPT_TEMPLATE = """
This is the test file you previously wrote, but there are error messages as follows. 
code:
```python
{test_code}
```
Error message:
{error_message}

The test file is at the same directory level as the source file (filename=core.py).
Use 'from core import ...' and 'from env import env' to import the source file.


You need to rewrite the entire test file to correct the errors. 
Note: output the complete file.


you should correctly import the necessary classes based on these file locations!
Return ```python your_code_here ``` with NO other texts,
your code:
"""