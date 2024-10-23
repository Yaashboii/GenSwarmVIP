import os
import json
import numpy as np
import matplotlib.pyplot as plt


class ExperimentAnalyzer:
    def __init__(self, experiment_path, tolerance=0.05):
        self.experiment_path = experiment_path
        self.tolerance = tolerance

    def analyze_result(self, run_result):
        analysis = {}  # 这里可以添加自定义的分析逻辑
        return analysis

    def plot_and_print_results(self, data, labels, ylabel, title, colors, save_filename, rotation=False,
                               success_conditions=None):
        if rotation:
            rotation = -90
        else:
            rotation = 0

        plt.figure(figsize=(10, 6))
        plt.bar(labels, data, color=colors)
        plt.xticks(rotation=rotation)
        plt.ylabel(ylabel)
        plt.title(title)

        if success_conditions:
            for condition in success_conditions:
                metric, operator, threshold = condition
                if metric == ylabel:
                    plt.axhline(y=threshold, color='r', linestyle='--', label=f"Success threshold ({threshold})")

        for i, v in enumerate(data):
            plt.text(i, v + 0.01, f"{v:.2f}", ha='center', rotation=rotation)

        if not os.path.exists(f"../workspace/{self.experiment_path}/pic"):
            os.makedirs(f"../workspace/{self.experiment_path}/pic")
        plt_path = os.path.join(f"../workspace/{self.experiment_path}/pic", save_filename)
        plt.tight_layout()
        plt.legend()
        plt.savefig(plt_path)
        plt.show()
        print(f"{title}: {data}")

    def save_experiment_result(self, path, result, analysis, run_code_result, retry_times):
        def convert_to_serializable(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            if isinstance(obj, dict):
                return {k: convert_to_serializable(v) for k, v in obj.items()}
            if isinstance(obj, list):
                return [convert_to_serializable(i) for i in obj]
            return obj

        serializable_result = convert_to_serializable(result)
        serializable_analysis = convert_to_serializable(analysis)
        combined_result = {
            'run_code_result': run_code_result,
            'retry_times': retry_times,
            'analysis': serializable_analysis,
            'experiment_data': serializable_result,
        }
        result_file = os.path.join(path, 'result.json')
        os.makedirs(path, exist_ok=True)
        with open(result_file, 'w') as f:
            json.dump(combined_result, f, indent=4)
