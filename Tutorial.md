# 仿真环境使用手册

## 1. 环境准备

### 1.1 建立工作空间

- 创建工作空间 并 建立包
- 包的名字必须为 ***code_llm***

```cd
mkdir -p 自定义空间名称/src
cd 自定义空间名称
catkin_make

cd src
catkin_create_pkg code_llm rospy std_msgs geometry_msgs message_generation  

cd code_llm
```

### 1.2 加入包文件

- 把Github上的 CodeLLM中全部文件复制到code_llm包中, 粘贴完如下图所示:

<img src="aserts/codellm.png" alt="codellm" style="zoom:67%;" />

### 1.3 编译工作空间

- 进入自定义工作空间目录, 编译

```
cd 工作空间
# 需要安装python3-em
sudo apt install python3-em

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3   # 需要根据实际的python解释器位置更改路径
```

### 1.4 运行仿真环境

- 编辑环境变量, 注意根据实际情况调整***工作空间***和***ros***的绝对路径

```
gedit ~/.bashrc
加入以下内容: 
export PYTHONPATH=/home/{替换你的用户名}/{替换工作空间名}/devel/lib/python3/dist-packages:$PYTHONPATH
export PYTHONPATH=/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH  # 根据实际的ubuntu版本进行调整
```

- 打开一个新的Terminal

```
cd 工作空间/src/code_llm/modules/env
conda activate pyxxx   # 选择使用哪个python来运行, 这里展示了conda的情况. py要3.10即可
python environment.py
```

### 1.5 配置pycharm中的路径

配置后可在 pycharm 中引用 rospy 以及编译的消息文件。
使用vscode可以将下面这段代码配置进settings.json中。

```json
{
  "python.autoComplete.extraPaths": [
    "/opt/ros/noetic/lib/python3/dist-packages",
    "/xxx/yyy工作空间/devel/lib/python3/dist-packages"
  ]
}
```

- 同样地, 手动给python interpreter添加***code_llm***和***ros***包的路径, 如图中倒数两行(added by user)所示:

<img src="aserts/path.png" alt="path" style="zoom:67%;" />

## 2. 运行

### 2.1 正常测试运行

- 运行 `environment.py` , 在Terminal中运行以下命令:

```
cd 工作空间/src/code_llm/modules/env
conda activate conda环境名 # 激活conda环境
python environment.py
```
> note: 注意在运行前，先运行 environment.py

- 运行 `code_llm/run.py` , 直接idea内部运行即可

### 2.2 对某一个已经生成的工作空间进行测试

- 运行 `environment.py` , 在Terminal中运行以下命令:
- 修改 `code_llm/modules/stages/running_stage.py`中的 
`set_workspace_root('/home/ubuntu/Desktop/CodeLLM/workspace/2024-03-05_20-03-52')`为想要测试的工作空间路径
- 运行 `code_llm/modules/stages/running_stage.py` , 直接idea内部运行即可


## 日志

---

Version: 1.0

Data: 2024/3/5

Editors: MiangChen

使用手册初稿，说明了如何建立工作空间，加入包文件，编译工作空间，运行仿真环境，配置pycharm中的路径。

---

Version: 1.1

Data: 2024/3/6

Editors: WenkangJi

增加了使用方法 以及对初稿内容进行补充说明

---
