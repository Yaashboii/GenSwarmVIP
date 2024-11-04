import json
import os
import pickle
import traceback

from modules.utils.root import root_manager


class CodeRunner:
    def __init__(
        self,
        time_out,
        target_pkl,
        script_name,
        feedback,
        experiment_path: str,
        env_manager=None,
    ):
        self.time_out = time_out
        self.target_pkl = target_pkl
        self.script_name = script_name
        self.feedback = feedback
        self.experiment_path = experiment_path
        self.env_manager = env_manager

    def run_code(self, experiment_id):
        experiment_path = os.path.join(self.experiment_path, experiment_id)
        root_manager.update_root(experiment_path)
        try:
            from modules.framework.actions.run_code import runcode

            result = runcode(
                experiment_path=experiment_path,
                timeout=self.time_out,
                target_pkl=self.target_pkl,
                feedback=self.feedback,
                script=self.script_name,
                env_manager=self.env_manager,
            )
        except Exception as e:
            traceback.print_exc()
            print(f"Error in run_code: {e}")
            # result_queue.put({'source': 'run_code', 'error': True, 'reason': str(e)})

    def load_result(self, experiment_id):
        experiment_path = os.path.join(self.experiment_path, experiment_id)
        file_path = os.path.join(experiment_path, "wo_vlm.pkl")
        if os.path.exists(file_path):
            with open(file_path, "rb") as file:
                wo_vlm_result = pickle.load(file)
        else:
            wo_vlm_result = None

        vlm_file_path = os.path.join(experiment_path, "with_vlm.pkl")
        if os.path.exists(vlm_file_path):
            with open(vlm_file_path, "rb") as file:
                vlm_result = pickle.load(file)
        else:
            vlm_result = None
        return wo_vlm_result, vlm_result
