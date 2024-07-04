import radon.complexity as cc
from radon.metrics import mi_visit, mi_rank
import ast


class CodeAnalyzer:
    """
    CodeAnalyzer 类用于分析给定代码的复杂度和可维护性。

    循环复杂度 (Cyclomatic Complexity, CC):
    ----------------------------------------
    循环复杂度是一种用于衡量程序复杂度的度量标准，由 Thomas J. McCabe 于 1976 年提出。它通过计算程序中的独立路径数量来评估代码的复杂度。具体来说，循环复杂度反映了代码中控制流的复杂程度，包括条件分支、循环等结构。

    较高的循环复杂度通常表明代码可能包含更多的条件和分支，因而更难以理解、测试和维护。反之，较低的循环复杂度则意味着代码较为简单和直线化，通常更容易维护。

    可维护性指数 (Maintainability Index, MI):
    ------------------------------------------
    可维护性指数是一种用于评估代码可维护性的度量标准，综合考虑了代码的循环复杂度、行数和注释率等因素。可维护性指数通常在 0 到 100 之间，得分越高表示代码越容易维护。

    计算可维护性指数的公式通常如下：

    MI = 171 - 5.2 * log(Halstead Volume) - 0.23 * (Cyclomatic Complexity) - 16.2 * log(Lines of Code) + 50 * sin(sqrt(2.46 * Percentage of Comments))

    其中 Halstead Volume 是通过 Halstead 复杂度度量计算得出的，Lines of Code 表示代码行数，Percentage of Comments 表示注释率。高 MI 分数表明代码结构清晰、注释充分、复杂度适中，因此更容易维护。

    使用方法:
    ----------
    1. 初始化 CodeAnalyzer 实例时传入待分析的代码字符串。
    2. 调用 `calculate_cyclomatic_complexity` 方法计算代码的平均循环复杂度。
    3. 调用 `calculate_maintainability_index` 方法计算代码的可维护性指数。
    4. 调用 `analyze` 方法一次性获取代码的循环复杂度和可维护性指数。

    示例:
    -------
    analyzer = CodeAnalyzer(code_string)
    analysis_result = analyzer.analyze()
    print(analysis_result)
    """

    def __init__(self, code: str):
        self.code = code

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

    def analyze(self):
        avg_complexity = self.calculate_cyclomatic_complexity()
        mi_score = self.calculate_maintainability_index()
        return avg_complexity,mi_score


if __name__ == '__main__':
    # Example usage
    code = """
def example_function(x):
    if x > 0:
        return x
    else:
        return -x

def another_function(y):
    for i in range(y):
        print(i)
    """
    analyzer = CodeAnalyzer(code)
    analysis_result = analyzer.analyze()
    print(analysis_result)
