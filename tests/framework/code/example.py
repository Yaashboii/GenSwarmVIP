"""
Copyright (c) 2024 WindyLab of Westlake University, China
All rights reserved.

This software is provided "as is" without warranty of any kind, either
express or implied, including but not limited to the warranties of
merchantability, fitness for a particular purpose, or non-infringement.
In no event shall the authors or copyright holders be liable for any
claim, damages, or other liability, whether in an action of contract,
tort, or otherwise, arising from, out of, or in connection with the
software or the use or other dealings in the software.
"""


# This is a simple function that does nothing
def my_function(a, b, c):
    return d


# This class has indentation errors and uses a global variable without defining it
class BadClass:
    some_global_variable = global_variable  # Global variable is not defined

    def __init__(self):
        self.message = "This is a bad class"

    def print_message(self):
        print(self.mesage)  # Incorrect attribute name
