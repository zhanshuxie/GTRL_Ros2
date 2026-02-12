import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gtrl_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 拷贝 launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 拷贝 urdf/xacro
        (os.path.join('share', package_name, 'urdf/xacro/bringup'), glob('urdf/xacro/bringup/*')),

        (os.path.join('share', package_name, 'urdf/xacro/camera'), glob('urdf/xacro/camera/*.xacro')),  # 不能识别文件夹
        (os.path.join('share', package_name, 'urdf/xacro/camera/fisheye'), glob('urdf/xacro/camera/fisheye/*')),


        (os.path.join('share', package_name, 'urdf/xacro/laser'), glob('urdf/xacro/laser/*')),

        (os.path.join('share', package_name, 'urdf/xacro/robot/gazebo'), glob('urdf/xacro/robot/gazebo/*')),
        (os.path.join('share', package_name, 'urdf/xacro/robot/scout'), glob('urdf/xacro/robot/scout/*')),
        (os.path.join('share', package_name, 'urdf/xacro/robot/wheels'), glob('urdf/xacro/robot/wheels/*')),

        # 拷贝 world
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        # 拷贝 rviz
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # 拷贝 meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')),
        (os.path.join('share', package_name, 'meshes/laser'), glob('meshes/laser/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xzs',
    maintainer_email='3246162287@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'train_node = gtrl_ros2.SAC.main:main',
            'teleop_node = gtrl_ros2.DIL.keyboard_control:main',
        ],
    },
)
