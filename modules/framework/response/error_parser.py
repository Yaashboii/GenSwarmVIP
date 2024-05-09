from modules.framework.code.function_tree import FunctionTree
from modules.framework.error import GrammarError
from modules.framework.code.grammer_checker import GrammarChecker


class ErrorParser():
    def __init__(self):
        super().__init__()
        self._imports: set[str] = set()
        self._function_tree = FunctionTree()
        self._grammar_checker = GrammarChecker()

    def parse(self, function_names):
        for function_name in function_names:
            self._check_function_grammar(function_name)
            self._check_caller_function_grammar(function_name)

    def _check_function_grammar(self, function_name):
        #     self._save_by_function(self._function_tree[function_name])

        errors = self._grammar_checker.check_code_errors(self._file.file_path)
        status = 'passed' if errors else 'failed'
        raise GrammarError(message=f'Grammar check {status} for {function_name}',
                           grammar_error=errors)

    def _check_caller_function_grammar(self, function_name):
        [self._check_function_grammar(f.name)
         for f in self._function_tree[function_name].callers]
