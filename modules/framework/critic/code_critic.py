from modules.framework.critic.critic import Critic


class CodeErrorCritic(Critic):
    def evaluate(self, code) -> (bool, str):
        """
        Specific implementation for code error evaluation.

        Returns:
            str: Suggestions or feedback based on code error evaluation.
        """
        self._suggestion = "No code errors found."
        self._is_passed = True

        self._exec_safe(code, {}, {})
        self._logger.debug("Checking for code errors...")
        self._logger.debug(self._suggestion)
        return self._is_passed, self._suggestion
    
    def _exec_safe(self, code_str, gvars, lvars):
        banned_phrases = ['import', '__']
        for phrase in banned_phrases:
            assert phrase not in code_str
        
        empty_fn = lambda *args, **kwargs: None
        custom_gvars = self._merge_dicts([
            gvars,
            {'exec': empty_fn, 'eval': empty_fn}
        ])
        try:
            exec(code_str, custom_gvars, lvars)
        except Exception as e:
            self._is_passed = False
            self._suggestion = e

    def _merge_dicts(self, dicts):
        return {
            k : v 
            for d in dicts
            for k, v in d.items()
        }
    
if __name__ == '__main__':
    _code_critic = CodeErrorCritic()
    print("================================================")
    try:
        _code_critic.evaluate("c = a+b")
    except Exception as e:
        print(e)