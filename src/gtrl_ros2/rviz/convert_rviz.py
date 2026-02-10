import sys
import os

# ROS 1 到 ROS 2 的类名映射表
REPLACEMENTS = {
    # Panels (面板) - 改为 rviz_common
    "Class: rviz/Displays": "Class: rviz_common/Displays",
    "Class: rviz/Selection": "Class: rviz_common/Selection",
    "Class: rviz/Tool Properties": "Class: rviz_common/Tool Properties",
    "Class: rviz/Views": "Class: rviz_common/Views",
    "Class: rviz/Time": "Class: rviz_common/Time",
    
    # Displays (显示项) - 改为 rviz_default_plugins
    "Class: rviz/Grid": "Class: rviz_default_plugins/Grid",
    "Class: rviz/RobotModel": "Class: rviz_default_plugins/RobotModel",
    "Class: rviz/TF": "Class: rviz_default_plugins/TF",
    "Class: rviz/LaserScan": "Class: rviz_default_plugins/LaserScan",
    "Class: rviz/Map": "Class: rviz_default_plugins/Map",
    "Class: rviz/Odometry": "Class: rviz_default_plugins/Odometry",
    "Class: rviz/Image": "Class: rviz_default_plugins/Image",
    "Class: rviz/Camera": "Class: rviz_default_plugins/Camera",
    "Class: rviz/PointCloud2": "Class: rviz_default_plugins/PointCloud2",
    "Class: rviz/MarkerArray": "Class: rviz_default_plugins/MarkerArray",
    "Class: rviz/Axes": "Class: rviz_default_plugins/Axes",
    "Class: rviz/Path": "Class: rviz_default_plugins/Path",
    
    # Tools (工具栏)
    "Class: rviz/Interact": "Class: rviz_default_plugins/Interact",
    "Class: rviz/MoveCamera": "Class: rviz_default_plugins/MoveCamera",
    "Class: rviz/Select": "Class: rviz_default_plugins/Select",
    "Class: rviz/FocusCamera": "Class: rviz_default_plugins/FocusCamera",
    "Class: rviz/Measure": "Class: rviz_default_plugins/Measure",
    "Class: rviz/SetInitialPose": "Class: rviz_default_plugins/SetInitialPose",
    "Class: rviz/SetGoal": "Class: rviz_default_plugins/SetGoal",
    "Class: rviz/PublishPoint": "Class: rviz_default_plugins/PublishPoint",
    
    # View Controllers (视角控制) - 关键！保留视角的重点
    "Class: rviz/Orbit": "Class: rviz_default_plugins/Orbit",
    "Class: rviz/TopDownOrtho": "Class: rviz_default_plugins/TopDownOrtho",
    "Class: rviz/ThirdPersonFollower": "Class: rviz_default_plugins/ThirdPersonFollower",
    "Class: rviz/XYOrbit": "Class: rviz_default_plugins/XYOrbit"
}

def convert_rviz(file_path):
    if not os.path.exists(file_path):
        print(f"Error: File '{file_path}' not found.")
        return

    with open(file_path, 'r') as f:
        content = f.read()

    # 执行替换
    new_content = content
    for old_str, new_str in REPLACEMENTS.items():
        new_content = new_content.replace(old_str, new_str)

    # 保存为新文件，防止覆盖原文件出错
    new_file_path = file_path.replace(".rviz", "_ros2.rviz")
    
    with open(new_file_path, 'w') as f:
        f.write(new_content)
    
    print(f"Success! Converted file saved as: {new_file_path}")
    print("Use this new file in your launch script.")

if __name__ == "__main__":
    # 这里填写你的文件名
    convert_rviz("scout.rviz")