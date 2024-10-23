import os
import traceback

class CodeRunner:
    def __init__(self, experiment_duration, target_pkl, script_name):
        self.experiment_duration = experiment_duration
        self.target_pkl = target_pkl
        self.script_name = script_name

    def run_code(self, experiment, result_queue):
        experiment_path = os.path.join(self.experiment_path, experiment)

        try:
            result = runcode(
                experiment_path=experiment_path,
                timeout=self.experiment_duration - 3,
                target_pkl=self.target_pkl,
                feedback='VLM',
                script=self.script_name
            )

            if result.get("success"):
                result_queue.put({'source': 'run_code', 'error': False, 'reason': ''})
            else:
                result_queue.put({'source': 'run_code', 'error': True, 'reason': result.get("error_message", "")})

        except Exception as e:
            traceback.print_exc()
            print(f"Error in run_code: {e}")
            result_queue.put({'source': 'run_code', 'error': True, 'reason': str(e)})
