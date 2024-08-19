import radon.complexity as cc
from radon.metrics import mi_visit
import ast

from modules.framework.parser import CodeParser


class CodeAnalyzer:
    """
    CodeAnalyzer 类用于分析给定代码的复杂度和可维护性。
    """

    def __init__(self, code: str):
        self.code = code
        self.parser = CodeParser()
        self.parser.parse_code(code)

    def calculate_cyclomatic_complexity(self):
        blocks = cc.cc_visit(self.code)
        complexities = [block.complexity for block in blocks]
        if complexities:
            average_complexity = sum(complexities) / len(complexities)
        else:
            average_complexity = 0
        return average_complexity

    def calculate_maintainability_index(self):
        mi_score = mi_visit(self.code, False)
        return mi_score

    def count_lines_and_comments(self):
        total_lines = 0
        comment_lines = 0
        code_lines = 0
        in_multiline_comment = False
        lines = self.code.split('\n')
        for line in lines:
            stripped_line = line.strip()
            if not stripped_line:
                continue
            total_lines += 1
            if in_multiline_comment:
                comment_lines += 1
                if stripped_line.endswith("'''") or stripped_line.endswith('"""'):
                    in_multiline_comment = False
            elif stripped_line.startswith('#'):
                comment_lines += 1
            elif stripped_line.startswith("'''") or stripped_line.startswith('"""'):
                comment_lines += 1
                if not (stripped_line.endswith("'''") or stripped_line.endswith('"""') and len(stripped_line) > 3):
                    in_multiline_comment = True
            else:
                code_lines += 1

        comment_ratio = comment_lines / total_lines if total_lines > 0 else 0
        return total_lines, code_lines, comment_lines, comment_ratio

    def analyze_functions(self):
        function_stats = {}
        for func_name, func_code in self.parser.function_dict.items():
            func_analyzer = CodeAnalyzer(func_code)
            total_lines, code_lines, comment_lines, comment_ratio = func_analyzer.count_lines_and_comments()
            function_stats[func_name] = {
                "total_lines": total_lines,
                "code_lines": code_lines,
                "comment_lines": comment_lines,
                "comment_ratio": comment_ratio
            }
        return function_stats

    def analyze(self):
        avg_complexity = self.calculate_cyclomatic_complexity()
        mi_score = self.calculate_maintainability_index()
        total_lines, code_lines, comment_lines, comment_ratio = self.count_lines_and_comments()
        function_stats = self.analyze_functions()
        return {
            "average_complexity": avg_complexity,
            "maintainability_index": mi_score,
            "total_lines": total_lines,
            "code_lines": code_lines,
            "comment_lines": comment_lines,
            "comment_ratio": comment_ratio,
            "function_stats": function_stats
        }


if __name__ == '__main__':
    # Example usage
    code = """
def example_function(x):
    # This is a comment
    '''

    '''
    if x > 0:
        return x
    else:
        return -x

def another_function(y):
    \"\"\"This is a docstring comment\"\"\"
    for i in range(y):
        print(i)
    """
    analyzer = CodeAnalyzer(code)
    analysis_result = analyzer.analyze()
    print(analysis_result)