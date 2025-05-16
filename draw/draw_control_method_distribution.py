import os
import re
import matplotlib.pyplot as plt

# Define the folder path
base_path = "../workspace/encircling"

# Data structures for results
folder_summary = {}

# Traverse all subfolders (simulate date-based folders like "2025-05-13_04-06-41")
if os.path.exists(base_path):
    for folder_name in os.listdir(base_path):
        folder_path = os.path.join(base_path, folder_name)
        if os.path.isdir(folder_path):
            global_count = 0
            local_count = 0
            classification = {}
            for subfolder_name in os.listdir(folder_path):
                subfolder_path = os.path.join(folder_path, subfolder_name)
                if os.path.isdir(subfolder_path):
                    global_skills_path = os.path.join(subfolder_path, "global_skill.py")
                    if os.path.exists(global_skills_path):
                        with open(global_skills_path, "r", encoding="utf-8") as f:
                            content = f.read()
                            has_only_import = (
                                    re.search(r'^from\s+global_apis\s+import\s+.+$', content, re.M) is not None and
                                    re.search(r'^\s*def\s+', content, re.M) is None
                            )
                            if has_only_import:
                                classification[subfolder_name] = 'local'
                                local_count += 1
                            else:
                                classification[subfolder_name] = 'global'
                                global_count += 1
            if global_count + local_count > 0:
                folder_summary[folder_name] = {
                    'global': global_count,
                    'local': local_count,
                    'classification': classification
                }

# Plotting the summary chart
fig, ax = plt.subplots()
labels = list(folder_summary.keys())
global_vals = [folder_summary[k]['global'] for k in labels]
local_vals = [folder_summary[k]['local'] for k in labels]

x = range(len(labels))
ax.bar(x, global_vals, label='Global')
ax.bar(x, local_vals, bottom=global_vals, label='Local')
ax.set_xticks(x)
ax.set_xticklabels(labels, rotation=45, ha='right')
ax.set_ylabel('Count')
ax.set_title('Global vs Local Skills per Date Folder')
ax.legend()

plt.tight_layout()
plt.show()
