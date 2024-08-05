import os


def extra_exp(base_path, out_type="dir"):
    if out_type == "dir":
        return [
            os.path.join(base_path, item)
            for item in os.listdir(base_path)
            # if os.path.isdir(os.path.join(base_path, item)) and os.path.exists(
            #     os.path.join(base_path, item, "human_feedback.txt"))
        ]
    elif out_type == "name":

        return [
            item
            for item in os.listdir(base_path)
            # if os.path.isdir(os.path.join(base_path, item)) and os.path.exists(
            #     os.path.join(base_path, item, "human_feedback.txt"))
        ]
