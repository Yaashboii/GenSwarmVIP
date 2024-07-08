import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


class LogAnalyzer:
    def __init__(self, base_path):
        self.base_path = base_path
        self.data = {
            'parallel': [],
            'sequential': [],
            'layer': []
        }
        self.patterns = {
            'Generate functions cost time': re.compile(r"Generate functions cost time: (\d+\.\d+)"),
            'average complexity': re.compile(r"average complexity: (\d+\.\d+), mi score: (\d+\.\d+)")
        }

    def analyze(self):
        # Traverse the directory structure
        for category in self.data.keys():
            category_path = os.path.join(self.base_path, category)
            if os.path.exists(category_path):
                for task in os.listdir(category_path):
                    task_path = os.path.join(category_path, task)
                    for experiment in os.listdir(task_path):
                        experiment_path = os.path.join(task_path, experiment)
                        if os.path.isdir(experiment_path):
                            log_file_path = os.path.join(experiment_path, 'log.md')
                            if os.path.exists(log_file_path):
                                self._process_log_file(log_file_path, category)

    def _process_log_file(self, log_file_path, category):
        with open(log_file_path, 'r', encoding='utf-8') as file:
            log_contents = file.read()

        extracted_data = {key: [] for key in self.patterns.keys()}
        extracted_data['mi score'] = []
        for key, pattern in self.patterns.items():
            matches = pattern.findall(log_contents)
            if key == 'average complexity':
                for match in matches:
                    extracted_data['average complexity'].append(float(match[0]))
                    extracted_data['mi score'].append(float(match[1]))
            else:
                extracted_data[key].extend([float(match) for match in matches])

        self.data[category].append(extracted_data)

    def calculate_statistics(self):
        stats = {}
        for category, data_list in self.data.items():
            if data_list:
                category_data = {key: np.array([item[key] for item in data_list if item[key]]) for key in self.patterns.keys()}
                category_data['mi score'] = np.array([item['mi score'] for item in data_list if item['mi score']])

                count = {key: len(values) for key, values in category_data.items()}
                mean = {key: np.mean(values) if len(values) > 0 else np.nan for key, values in category_data.items()}
                std = {key: np.std(values) if len(values) > 0 else np.nan for key, values in category_data.items()}
                success_rate = {key: np.mean(values != 0) * 100 if len(values) > 0 else np.nan for key, values in category_data.items()}

                stats[category] = {
                    'count': count,
                    'mean': mean,
                    'std': std,
                    'success_rate': success_rate
                }
        return stats

    def plot_statistics(self, stats):
        if not stats:
            print("No data found to plot.")
            return

        metrics = ['Generate functions cost time', 'average complexity', 'mi score']
        categories = list(stats.keys())

        mean_fig, mean_ax = plt.subplots(figsize=(10, 6))
        std_fig, std_ax = plt.subplots(figsize=(10, 6))

        x = np.arange(len(metrics))
        width = 0.2

        for i, category in enumerate(categories):
            mean_values = [stats[category]['mean'][metric] for metric in metrics]
            std_values = [stats[category]['std'][metric] for metric in metrics]

            mean_ax.bar(x + i * width, mean_values, width, label=category)
            std_ax.bar(x + i * width, std_values, width, label=category)

        mean_ax.set_title('Mean Values')
        mean_ax.set_ylabel('Mean')
        mean_ax.set_xticks(x + width / 2)
        mean_ax.set_xticklabels(metrics)
        mean_ax.legend()

        std_ax.set_title('Standard Deviation Values')
        std_ax.set_ylabel('Standard Deviation')
        std_ax.set_xticks(x + width / 2)
        std_ax.set_xticklabels(metrics)
        std_ax.legend()

        plt.tight_layout()
        plt.show()


# Example usage
base_path = '../workspace'
analyzer = LogAnalyzer(base_path)
analyzer.analyze()
stats = analyzer.calculate_statistics()

if stats:
    analyzer.plot_statistics(stats)
else:
    print("No statistical data available.")
