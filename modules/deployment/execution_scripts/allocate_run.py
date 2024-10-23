import os
import pickle
import sys
import json
from global_skill import allocate_run
import rospy
import numpy as np


def main():
    result = allocate_run()
    # 将返回值（字典）保存为 JSON 文件
    with open('allocate_result.pkl', 'wb') as f:
        pickle.dump(result, f)
    print(result)


if __name__ == "__main__":
    print(sys.argv)
    rospy.init_node('allocate_run_node', anonymous=True)

    main()
