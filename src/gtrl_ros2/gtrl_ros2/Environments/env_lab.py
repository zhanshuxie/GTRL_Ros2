#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
"""
Created on Mon Aug 21 16:17:01 2023

@author: oscar
"""

import os
from os import path

import time
import math
import random
import numpy as np
from numpy import inf
from collections import deque
from squaternion import Quaternion

import cv2

# [ROS 2 修改] 引入 rclpy 和相关库
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import subprocess
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
# [ROS 2 修改] point_cloud2 工具通常在 sensor_msgs_py 中
import sensor_msgs_py.point_cloud2 as pc2 
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import Image, LaserScan, PointCloud2

from gazebo_msgs.msg import EntityState # 服务的request是EntityState类对象
from gazebo_msgs.srv import SetEntityState  # 通过服务来设置'实体状态'

from ament_index_python.packages import get_package_share_directory

# import ros_numpy # [ROS 2 修改] 移除未使用的 ros_numpy

os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

# Check if the random goal position is located on an obstacle and do not accept it if it is
def check_pos(x, y):
    goalOK = False

    if -6.7 < x < -6.5 and -3.4 < y < 3.6:
        goalOK = True
    
    elif 3.5 < x < 5.2 and -3.4 < y < 3.6:
        goalOK = True
        
    elif -6.4 < x < 3.5 and -3.4 < y <-2.8:
        goalOK = True
    
    elif -6.4 < x < 3.5 and 3.1 < y < 3.6:
        goalOK = True
    
    elif -1.8 < x < -1.4 and -3.4 < y < 3.6:
        goalOK = True

    return goalOK


# Function to put the laser data in bins
def binning(lower_bound, data, quantity):
    width = round(len(data) / quantity)
    quantity -= 1
    bins = []
    for low in range(lower_bound, lower_bound + quantity * width + 1, width):
        bins.append(min(data[low:low + width]))
    return np.array([bins])


class GazeboEnv:
    """Superclass for all Gazebo environments.
    """
    metadata = {'render.modes': ['human']}

    # [ROS 2 修改] 移除了 ROS_MASTER_URI 参数
    def __init__(self, launchfile, height, width, nchannels):

        self.odomX = 0
        self.odomY = 0

        self.goalX = 1.0
        self.goalY = 0.0

        self.upper = 10.0
        self.lower = -10.0
        self.velodyne_data = np.ones(20) * 10
        self.last_laser = None
        self.last_odom = None
        self.last_image = None
        self.last_image_fish = None
        self.rgb_image = None
        self.original_image = None
        self.collision = 0
        self.last_act = 0
        
        # [ROS 2 修改] 增加接收标志位，替代 wait_for_message
        self.new_odom = False
        self.new_laser = False
        self.new_image = False
        self.new_image_fish = False

        self.x_pos_list = deque(maxlen=5)
        self.y_pos_list = deque(maxlen=5)

        self.set_self_state = EntityState()
        self.set_self_state.name = 'scout'

        self.set_self_state.pose.position.x = 0.
        self.set_self_state.pose.position.y = 0.
        self.set_self_state.pose.position.z = 0.
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
        self.gaps = [[-1.6, -1.57 + 3.14 / 20]]
        for m in range(19):
            self.gaps.append([self.gaps[m][1], self.gaps[m][1] + 3.14 / 20])
        self.gaps[-1][-1] += 0.03

        # [ROS 2 修改] 移除 roscore 启动，初始化 ROS 2 节点
        # 检查是否已经初始化，main.py 通常会初始化
        if not rclpy.ok():
            rclpy.init()
        
        self.node = rclpy.create_node('gazebo_gym_env')
        print("ROS 2 Node initialized!")

        # [ROS 2 修改] Launch 启动逻辑
        # 尝试使用 ros2 launch。这里假设 launchfile 已经适配 ROS 2 (如 .launch.py 或 .xml)
        # 并且假设该 launch 文件位于 gtrl_ros2 包的 launch 目录下
        try:
            # 尝试获取包路径 (如果已 source install/setup.bash)
            pkg_share = get_package_share_directory('gtrl_ros2')
            launch_path = os.path.join(pkg_share, 'launch', launchfile)
        except Exception:
            # 如果找不到包 (例如在开发环境中)，尝试构建本地路径
            launch_path = os.path.join(os.getcwd(), 'src/gtrl_ros2/launch', launchfile)
            if not path.exists(launch_path):
                 # 回退到旧路径逻辑 (仅作参考，建议使用标准 colcon 结构)
                 launch_path = os.path.join('/home/oscar/ws_oscar/DRL-Transformer-SimtoReal-Navigation/catkin_ws/src/gtrl/launch', launchfile)

        if not path.exists(launch_path) and not launchfile.startswith("/"):
             # 如果不是绝对路径且找不到，可能直接传给 ros2 launch 命令处理
             print(f"Warning: Launch file path {launch_path} check failed. Trying direct execution.")
             pass
             
        time.sleep(2) # ROS 2 启动不需要等 roscore，但给一点缓冲

        # 注意：这里使用 subprocess 启动 launch。在 ROS 2 中通常建议独立运行 Gazebo，或者使用 LaunchService。
        # 为了保持代码结构，这里仍然作为子进程启动。
        # 您可能需要根据实际情况调整命令，例如 ["ros2", "launch", "gtrl_ros2", launchfile]
        print(f"Launching: {launch_path}")
        # self.launch_process = subprocess.Popen(["ros2", "launch", launch_path])
        print("Gazebo launch command executed (Assuming external launch or updated command)")

        self.gzclient_pid = 0

        # [ROS 2 修改] Publishers & Subscribers
        self.vel_pub = self.node.create_publisher(Twist, '/scout/cmd_vel', 1)

        
        # Services (Clients)
        self.set_state = self.node.create_client(SetEntityState, '/set_entity_state')  # 用于初始化机器人的起点
        self.unpause = self.node.create_client(Empty, '/unpause_physics')
        self.pause = self.node.create_client(Empty, '/pause_physics')
        self.reset_proxy = self.node.create_client(Empty, '/reset_world')
        
        topic = 'vis_mark_array'
        self.publisher = self.node.create_publisher(MarkerArray, topic, 3)
        topic2 = 'vis_mark_array2'
        self.publisher2 = self.node.create_publisher(MarkerArray, topic2, 1)
        topic3 = 'vis_mark_array3'
        self.publisher3 = self.node.create_publisher(MarkerArray, topic3, 1)
        topic4 = 'vis_mark_array4'
        self.publisher4 = self.node.create_publisher(MarkerArray, topic4, 1)
        
        # 
        self.velodyne = self.node.create_subscription(PointCloud2, '/velodyne_points', self.velodyne_callback, 1)
        # 2D 激光雷达
        self.laser = self.node.create_subscription(LaserScan, '/front_laser/scan', self.laser_callback, 1)
        # self.odom进行了修改
        self.odom = self.node.create_subscription(Odometry, '/scout/odom', self.odom_callback, 1)
        # self.image是否需要修改???
        self.image = self.node.create_subscription(Image, '/camera/rgb/image_raw', self.image_callback, 1)
        # self.image_fish进行修改
        self.image_fish = self.node.create_subscription(Image, '/camera/fisheye/camera_fish/image_raw', self.image_fish_callback, 1)

    def seed(self, seed):
        random.seed(seed)
        np.random.seed(seed)
    
    # [ROS 2 新增] 同步调用服务的辅助函数
    def _call_service_sync(self, client, request, timeout_sec=30.0):
        # 判断服务是否上线
        waited = 0.0
        while not client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
            waited += 1.0
            if waited >= timeout_sec:
                service_name = client.srv_name if hasattr(client, 'srv_name') else 'unknown_service'
                raise TimeoutError(
                    f"Timeout waiting for service '{service_name}' for {timeout_sec:.1f}s. "
                    "Please ensure Gazebo is running and gazebo_ros plugins are loaded."
                )
        # 发送requst
        future = client.call_async(request)
        # spin等待服务处理完成
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    # Read velodyne pointcloud and turn it into distance data, then select the minimum value for each angle
    # range as state representation
    def velodyne_callback(self, v):
        # [ROS 2 修改] 使用 sensor_msgs_py.point_cloud2
        data = list(pc2.read_points(v, skip_nans=False, field_names=("x", "y", "z")))
        self.velodyne_data = np.ones(20) * 10
        for i in range(len(data)):
            if data[i][2] > -0.2:
                dot = data[i][0] * 1 + data[i][1] * 0
                mag1 = math.sqrt(math.pow(data[i][0], 2) + math.pow(data[i][1], 2))
                mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
                # 修复除零风险
                if mag1 * mag2 == 0:
                    beta = 0
                else:
                    val = dot / (mag1 * mag2)
                    val = max(min(val, 1.0), -1.0) # Clip for safety
                    beta = math.acos(val) * np.sign(data[i][1])  # * -1
                
                dist = math.sqrt(data[i][0] ** 2 + data[i][1] ** 2 + data[i][2] ** 2)

                for j in range(len(self.gaps)):
                    if self.gaps[j][0] <= beta < self.gaps[j][1]:
                        self.velodyne_data[j] = min(self.velodyne_data[j], dist)
                        break

    def laser_callback(self, scan):
        # 接收数据
        self.last_laser = scan
        # 标志位
        self.new_laser = True

    def odom_callback(self, od_data):
        self.last_odom = od_data
        self.new_odom = True

    def _ros_image_to_numpy(self, msg, desired_encoding="passthrough"):
        """将 sensor_msgs/Image 转换为 numpy 图像，避免对 cv_bridge 的二进制依赖。"""
        if msg is None:
            raise ValueError("Image message is None")

        channels_map = {
            "mono8": 1,
            "8UC1": 1,
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }

        src_encoding = msg.encoding.lower()
        if src_encoding not in channels_map:
            raise ValueError(f"Unsupported image encoding: {msg.encoding}")

        channels = channels_map[src_encoding]
        dtype = np.uint8

        image_np = np.frombuffer(msg.data, dtype=dtype)
        if channels == 1:
            image_np = image_np.reshape((msg.height, msg.width))
        else:
            image_np = image_np.reshape((msg.height, msg.width, channels))

        # 按请求编码转换
        if desired_encoding == "passthrough" or desired_encoding.lower() == src_encoding:
            return image_np.copy()

        if desired_encoding == "mono8":
            if channels == 1:
                return image_np.copy()
            if src_encoding == "rgb8":
                return cv2.cvtColor(image_np, cv2.COLOR_RGB2GRAY)
            if src_encoding == "bgr8":
                return cv2.cvtColor(image_np, cv2.COLOR_BGR2GRAY)
            if src_encoding == "rgba8":
                return cv2.cvtColor(image_np, cv2.COLOR_RGBA2GRAY)
            if src_encoding == "bgra8":
                return cv2.cvtColor(image_np, cv2.COLOR_BGRA2GRAY)

        if desired_encoding == "rgb8":
            if src_encoding == "rgb8":
                return image_np.copy()
            if src_encoding == "bgr8":
                return cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            if src_encoding == "bgra8":
                return cv2.cvtColor(image_np, cv2.COLOR_BGRA2RGB)
            if src_encoding == "rgba8":
                return cv2.cvtColor(image_np, cv2.COLOR_RGBA2RGB)

        raise ValueError(
            f"Unsupported conversion from {msg.encoding} to {desired_encoding}"
        )

    def image_callback(self, rgb_data):
        image = self._ros_image_to_numpy(rgb_data, "mono8")
        ######## Depth Image ##########
        # image = self.br.imgmsg_to_cv2(rgb_data, "passthrough")
        ######## RGB Image #########
        self.last_image = np.expand_dims(cv2.resize(image, (128, 64)), axis=2)
        self.new_image = True

    def image_fish_callback(self, rgb_data):
        # 灰度
        image = self._ros_image_to_numpy(rgb_data, "mono8")
        # 彩色
        self.original_image = self._ros_image_to_numpy(rgb_data, "rgb8")
        # 剪裁/缩放
        self.last_image_fish = np.expand_dims(cv2.resize(image[80:400, 140:500], (160, 128)), axis=2)
        image_ = self._ros_image_to_numpy(rgb_data, "rgb8")
        self.rgb_image = image_[80:400, 140:500, :]
        self.new_image_fish = True

    # Detect a collision from laser data
    def calculate_observation(self, data):
        min_range = 0.4
        min_laser = 2
        done = False
        col = False

        for i, item in enumerate(data.ranges):
            if min_laser > data.ranges[i]:
                min_laser = data.ranges[i]
            if (min_range > data.ranges[i] > 0.20):
                done = True
                col = True
                print(f'发生碰撞!!!距离:{data.ranges[i]}')

        # min_range = 0.4 
        # min_laser = 2
        # done = False
        # col = False

        # total_rays = len(data.ranges)
        # # 【修改2】只取正前方的扫描扇区（比如掐头去尾，只看中间 60% 的射线），防止扫到自己车身两侧的轮子
        # start_idx = int(total_rays * 0.2)
        # end_idx = int(total_rays * 0.8)

        # # 遍历时只看安全角度内的射线
        # for i in range(start_idx, end_idx):
        #     if min_laser > data.ranges[i]:
        #         min_laser = data.ranges[i]
        #     # 过滤掉 0.0 的无效点，如果有效点小于 0.16 米才算碰撞
        #     if (min_range > data.ranges[i] > 0.20):
        #         done = True
        #         col = True
        #         print(f'发生碰撞!!!距离:{data.ranges[i]}')
        
        return done, col, min_laser

    # Perform an action and read a new state
    def step(self, act, timestep):
        vel_cmd = Twist()
        vel_cmd.linear.x = float(act[0]) # Ensure float
        vel_cmd.angular.z = float(act[1])
        self.vel_pub.publish(vel_cmd)

        target = False
        
        # [ROS 2 修改] 服务调用
        # 让Gazebo从暂停状态继续计算,否则发布了速度,世界不动,传感器也不刷新
        self._call_service_sync(self.unpause, Empty.Request())

        time.sleep(0.1)

        # [ROS 2 修改] 等待数据刷新
        # 重置标志位
        self.new_odom = False
        self.new_laser = False
        self.new_image_fish = False
        # self.new_image = False # 代码逻辑似乎主要依赖 fisheye
        
        # 循环等待，直到收到所需消息
        # 添加超时保护防止死锁
        timeout = 5.0 
        start_wait = time.time()
        
        # 注意: 这里需要等待 Odom, Laser 和 FishEye 图像
        while not (self.new_odom and self.new_laser and self.new_image_fish):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_wait > timeout:
                print("Warning: Timeout waiting for sensor data in step()")
                break

        time.sleep(0.1)
        # 让Gazebo暂停
        self._call_service_sync(self.pause, Empty.Request())

        # 读取本步数据
        data = self.last_laser
        dataOdom = self.last_odom
        data_obs = self.last_image
        data_obs_fish = self.last_image_fish
        
        # 简单检查数据是否存在 (防止超时导致的 NoneType Error)
        if data is None or dataOdom is None:
            print("Error: Failed to retrieve sensor data.")
            # 返回当前状态或全零，防止程序崩溃
            # 这里简单处理，实际应根据需求处理
            return np.zeros((160, 128, 1)), 0, 0, 0, 0, 0, 0, True, np.zeros(4), False

        laser_state = np.array(data.ranges[:])
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        done, col, min_laser = self.calculate_observation(data)  # data是雷达数据

        # Calculate robot heading from odometry data
        # 更新机器人位姿odomX/odomY
        self.odomX = dataOdom.pose.pose.position.x
        self.odomY = dataOdom.pose.pose.position.y
        
        self.x_pos_list.append(round(self.odomX,2))
        self.y_pos_list.append(round(self.odomY,2))
        
        quaternion = Quaternion(
            dataOdom.pose.pose.orientation.w,
            dataOdom.pose.pose.orientation.x,
            dataOdom.pose.pose.orientation.y,
            dataOdom.pose.pose.orientation.z)
        euler = quaternion.to_euler(degrees=False)
        angle = round(euler[2], 4)

        # Calculate distance to the goal from the robot
        # 计算相对距离
        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        # Calculate the angle distance between the robots heading and heading toward the goal
        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY
        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        
        # Fix domain error for acos
        # beta2 航向误差
        val = dot / (mag1 * mag2)
        val = max(min(val, 1.0), -1.0)
        beta = math.acos(val)

        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
                
        beta2 = (beta - angle)
        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        # Publish visual data in Rviz
        # 目标点可视化
        markerArray = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.type = marker.CYLINDER  # 圆柱体 
        marker.action = marker.ADD
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.goalX  # 位置X
        marker.pose.position.y = self.goalY  # 位置Y
        marker.pose.position.z = 0.0

        markerArray.markers.append(marker)

        self.publisher.publish(markerArray)

        # 线速度动作强度条
        markerArray2 = MarkerArray()
        marker2 = Marker()
        marker2.header.frame_id = "odom"
        marker2.type = marker.CUBE
        marker2.action = marker.ADD
        marker2.scale.x = float(abs(act[0]))
        marker2.scale.y = 0.1
        marker2.scale.z = 0.01
        marker2.color.a = 1.0
        marker2.color.r = 1.0
        marker2.color.g = 0.0
        marker2.color.b = 0.0
        marker2.pose.orientation.w = 1.0
        marker2.pose.position.x = 5.0
        marker2.pose.position.y = 0.0
        marker2.pose.position.z = 0.0

        markerArray2.markers.append(marker2)
        self.publisher2.publish(markerArray2)
        
        # 角速度动作强度条
        markerArray3 = MarkerArray()
        marker3 = Marker()
        marker3.header.frame_id = "odom"
        marker3.type = marker.CUBE
        marker3.action = marker.ADD
        marker3.scale.x = float(abs(act[1]))
        marker3.scale.y = 0.1
        marker3.scale.z = 0.01
        marker3.color.a = 1.0
        marker3.color.r = 1.0
        marker3.color.g = 0.0
        marker3.color.b = 0.0
        marker3.pose.orientation.w = 1.0
        marker3.pose.position.x = 5.0
        marker3.pose.position.y = 0.2
        marker3.pose.position.z = 0.0

        markerArray3.markers.append(marker3)
        self.publisher3.publish(markerArray3)
        
        # 参考小方块(基准)
        markerArray4 = MarkerArray()
        marker4 = Marker()
        marker4.header.frame_id = "odom"
        marker4.type = marker.CUBE
        marker4.action = marker.ADD
        marker4.scale.x = 0.1
        marker4.scale.y = 0.1
        marker4.scale.z = 0.01
        marker4.color.a = 1.0
        marker4.color.r = 1.0
        marker4.color.g = 0.0
        marker4.color.b = 0.0
        marker4.pose.orientation.w = 1.0
        marker4.pose.position.x = 5.0
        marker4.pose.position.y = 0.4
        marker4.pose.position.z = 0.0

        markerArray4.markers.append(marker4)
        self.publisher4.publish(markerArray4)

        '''Bunch of different ways to generate the reward'''
        
        # 向目标点接近就满分
        r_heuristic = (self.distOld - Dist) * 20 #* math.cos(act[0]*act[1]/4)
        # 鼓励前进,抑制过大的角速度
        r_action = act[0]*2 - abs(act[1])
        # 抑制突变转向
        r_smooth = - abs(act[1] - self.last_act)/4  # 惩罚角速度突变(论文中没有)

        # 供下一step计算奖励使用      
        self.distOld = Dist

        r_target = 0.0
        r_collision = 0.0
        r_freeze = 0.0

        # Detect if the goal has been reached and give a large positive reward
        if Dist < 0.5:
            target = True
            done = True
            self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))
            r_target = 100

        # Detect if ta collision has happened and give a large negative reward
        if col:
            self.collision += 1
            r_collision = -100

        if timestep>10 and self.check_list(self.x_pos_list) and self.check_list(self.y_pos_list):
            r_freeze = -1

        reward = r_heuristic + r_action + r_collision + r_target + r_smooth #+ r_freeze
        # Dist [0, 1] beta2 [-1, 1]
        Dist  = min(Dist/15, 1.0) #max 15m away from current position
        beta2 = beta2 / np.pi
        toGoal = np.array([Dist, beta2, act[0], act[1]])
        
        ####### Depth Iamge ########
        # image = data_obs.copy()
        # image[np.isnan(image)] = 10.0
        # state = image/10
        
        ######## FishEye ####
        state = data_obs_fish / 255 # 原始图片每一个像素值是0～255的整数，归一化
        self.last_act = act[1]  # 计算 角速度突变的惩罚
        return state, r_heuristic, r_action, r_freeze, r_collision, r_target, reward, done, toGoal, target

    def check_list(self, buffer):
        it = iter(buffer)
        try:
            first = next(it)
        except StopIteration:
            return True
        return all((abs(first-x)<0.1) for x in buffer)

    def reset(self):
        # 每回合开始把环境重置成一个新的初始状态
        # 随机起点 + 随机目标 + 首帧采样

        # Resets the state of the environment and returns an initial observation.
        self._call_service_sync(self.reset_proxy, Empty.Request())

        # 随机朝向
        # 先随机一个yaw角
        angle = np.random.uniform(-np.pi, np.pi)
        # 再转成四元数
        quaternion = Quaternion.from_euler(0., 0., angle)
        # object_state 是 EntityState类对象
        object_state = self.set_self_state

        # 随机位置
        x = 0
        y = 0
        chk = False
        while not chk:
            x = np.random.uniform(-7.0, 7.0)
            y = np.random.uniform(-4.0, 4.0)
            # 判断是否可行
            chk = check_pos(x, y)
        object_state.pose.position.x = float(x)
        object_state.pose.position.y = float(y)
        object_state.pose.orientation.x = float(quaternion.x)
        object_state.pose.orientation.y = float(quaternion.y)
        object_state.pose.orientation.z = float(quaternion.z)
        object_state.pose.orientation.w = float(quaternion.w)
        
        # 通过 /set_entity_state 发布服务
        req = SetEntityState.Request()
        req.state = object_state
        self._call_service_sync(self.set_state, req)

        self.odomX = object_state.pose.position.x
        self.odomY = object_state.pose.position.y

        # 生成新目标点
        self.change_goal()
        # self.random_box()
        # 供下一个step奖励计算使用
        self.distOld = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        data = None
        data_obs = None
        data_obs_fish = None
        
        # 让Gazebo从暂停状态继续计算,否则发布了速度,世界不动,传感器也不刷新
        self._call_service_sync(self.unpause, Empty.Request())
        
        # [ROS 2 修改] 等待激光雷达数据
        self.new_laser = False
        self.new_image_fish = False
        
        timeout = 5.0
        start_wait = time.time()
        while not (self.new_laser and self.new_image_fish):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if time.time() - start_wait > timeout:
                print("Warning: Timeout waiting for data in reset()")
                break
                
        data = self.last_laser
        data_obs_fish = self.last_image_fish
        
        if data is None or data_obs_fish is None:
             # 安全回退
             return np.zeros((160, 128, 1)), np.zeros(4)

        laser_state = np.array(data.ranges[:])
        laser_state[laser_state == inf] = 10
        laser_state = binning(0, laser_state, 20)
        
        ##### FishEye Image ####
        camera_image = data_obs_fish
        # 转灰度图 mono8
        image = camera_image
        # 裁剪[80:400, 140:500]
        # resize到(160, 128)
        # 增加channel维
        image = np.expand_dims(cv2.resize(image[80:400, 140:500], (160, 128)), axis=2)
        # 归一化 shape(128, 160, 1) (HWC)
        state = image/255
        

        ######## Depth Image ##########
        # image = self.br.imgmsg_to_cv2(camera_image, "passthrough")
        # image = image.copy()
        # image[np.isnan(image)] = 10.0
        # image = np.expand_dims(cv2.resize(image, (128, 64)), axis=2)
        # state = image/10

        # 让Gazebo暂停
        self._call_service_sync(self.pause, Empty.Request())
        # 相对距离
        Dist = math.sqrt(math.pow(self.odomX - self.goalX, 2) + math.pow(self.odomY - self.goalY, 2))

        skewX = self.goalX - self.odomX
        skewY = self.goalY - self.odomY

        dot = skewX * 1 + skewY * 0
        mag1 = math.sqrt(math.pow(skewX, 2) + math.pow(skewY, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        
        val = dot / (mag1 * mag2)
        val = max(min(val, 1.0), -1.0)
        beta = math.acos(val)
        # 航向误差
        if skewY < 0:
            if skewX < 0:
                beta = -beta
            else:
                beta = 0 - beta
        beta2 = (beta - angle)

        if beta2 > np.pi:
            beta2 = np.pi - beta2
            beta2 = -np.pi - beta2
        if beta2 < -np.pi:
            beta2 = -np.pi - beta2
            beta2 = np.pi - beta2

        # Dist [0, 1] beta2 [-1, 1]
        Dist  = min(Dist/15, 1.0) # max 15m away from current position
        beta2 = beta2 / np.pi
        toGoal = np.array([Dist, beta2, 0.0, 0.0])  # 初始化时没有动作 动作置0
        return state, toGoal

    # Place a new goal and check if its lov\cation is not on one of the obstacles
    def change_goal(self):
        if self.upper < 10:
            self.upper += 0.008
        if self.lower > -10:
            self.lower -= 0.008

        gOK = False

        while not gOK:
            self.goalX = self.odomX + random.uniform(self.upper, self.lower)
            self.goalY = self.odomY + random.uniform(self.upper, self.lower)

            euclidean_dist = math.sqrt((self.goalX - self.odomX)**2 + (self.goalY - self.odomY)**2)
            if self.upper > 4 and euclidean_dist < 3:
                gOK = False
                continue
            elif self.upper > 8 and euclidean_dist < 6:
                gOK = False
                continue

            gOK = check_pos(self.goalX, self.goalY)
                
    # Randomly change the location of the boxes in the environment on each reset to randomize the training environment
    def random_box(self):
        for i in range(2):
            name = 'cardboard_box_' + str(i)

            x = 0
            y = 0
            chk = False
            while not chk:
                x = np.random.uniform(-6, 6)
                y = np.random.uniform(-6, 6)
                chk = check_pos(x, y)
                d1 = math.sqrt((x - self.odomX) ** 2 + (y - self.odomY) ** 2)
                d2 = math.sqrt((x - self.goalX) ** 2 + (y - self.goalY) ** 2)
                if d1 < 1.5 or d2 < 1.5:
                    chk = False
            box_state = EntityState()
            box_state.name = name
            box_state.pose.position.x = x
            box_state.pose.position.y = y
            box_state.pose.position.z = 0.
            box_state.pose.orientation.x = 0.0
            box_state.pose.orientation.y = 0.0
            box_state.pose.orientation.z = 0.0
            box_state.pose.orientation.w = 1.0
            self.set_state.publish(box_state)
