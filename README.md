# GoT-GTRL

## :book: Goal-guided Transformer-enabled Reinforcement Learning for Efficient Autonomous

[**Source**](https://github.com/OscarHuangWind/DRL-Transformer-SimtoReal-Navigation):Ros noetic --> Ros humble :zap:

[**Paper**](https://ieeexplore.ieee.org/document/10254445)

:sunglasses: Realized in ROS Gazebo simulator with Ubuntu 22.04, Ros humble, and Pytorch.

## User Guidance

### Create a new Virtual environment(conda is suggested).

```bash
conda create -n gtrl_ros python=3.10.12
```

### Activate virtual environment.

```bash
conda activate gtrl_ros2
```

### Install Dependencies.

```bash
pip install numpy matplotlib tqdm natsort cpprb pyyaml einops opencv-python squaternion lxml
```

### Clone the repository.
cd to your workspace and clone the repo.
```bash
git clone https://github.com/zhanshuxie/GTRL_Ros2.git
```

### Compile the workspace and clone the repo.
```bash
cd ~/$your workspace/GTRL_Ros2
colcon build --symlink-install
```

### Source the workspace.
```bash
source install/setup.bash
```

### Launch the simulation environment.
```bash
ros2 launch gtrl_ros2 simulation.launch.py
```

### Time to train!

:warning: you should make sure to launch the simulation environment first.

option1: run the SAC training script directly. 
```bash
cd ~/$your workspace/GTRL_Ros2/src/gtrl_ros2/gtrl_ros2/SAC
python3 main.py
```

option2: run the ROS node that contains the SAC training script. 
```bash
ros2 run gtrl_ros2 train_node
```

### Evaluate the trained model.

:warning: you should make sure to launch the simulation environment first.

option1: run the evaluation script directly. 
```bash
cd ~/$your workspace/GTRL_Ros2/src/gtrl_ros2/gtrl_ros2/SAC
python3 run_eval.py
```

option2: run the ROS node that contains the evaluation script. 
```bash
ros2 run gtrl_ros2 eval_node
```

### Collect demonstration data.

:warning: you should make sure to launch the simulation environment first.
source the workspace.
```bash
cd ~/$your workspace/GTRL_Ros2
source install/setup.bash
```
preprare to teleop the robot with keyboard control.
```bash
cd ~/$your workspace/GTRL_Ros2/src/gtrl_ros2/gtrl_ros2/DIL
python3 keyboard_control.py
```

then run the demonstration collection script.
```bash
cd ~/$your workspace/GTRL_Ros2/src/gtrl_ros2/gtrl_ros2/DIL
python3 demonstration.py
```