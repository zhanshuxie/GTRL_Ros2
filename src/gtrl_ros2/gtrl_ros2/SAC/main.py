#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 21 15:10:17 2023

@author: oscar
"""

import sys
import os
import time
import glob
import yaml
import statistics
import numpy as np
from tqdm import tqdm
from natsort import natsorted
from collections import deque
import matplotlib.pyplot as plt
from cpprb import PrioritizedReplayBuffer
# from sklearn.metrics import mean_squared_error

import torch
import torch.nn as nn
import torch.nn.functional as F

# 导入自定义的 SAC 算法和 Gazebo 环境
from gtrl_ros2.SAC.DRL import SAC
from gtrl_ros2.Environments.env_lab import GazeboEnv

# [ROS 2 修改] 引入 ROS 2 的 Python 库 rclpy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def evaluate(network, env, eval_episodes=10, epoch=0, node=None):
    """
    评估网络性能的函数。
    
    参数:
        network: 训练好的策略网络
        env: 环境对象 (需要传入，因为全局变量在函数中可能不适用)
        eval_episodes: 评估的轮数
        epoch: 当前训练的代数 (Epoch)
        node: [ROS 2 新增] ROS 2 节点对象，用于处理回调
    """
    obs_list = deque(maxlen=frame_stack) # 用于堆叠帧的队列
    env.collision = 0
    ep = 0
    avg_reward_list = []
    
    while ep < eval_episodes:
        count = 0
        obs, goal = env.reset() # 重置环境
        done = False
        avg_reward = 0.0

        # 初始化帧堆叠，复制第一帧4次
        for i in range(4):
            obs_list.append(obs)

        observation = np.concatenate((obs_list[-4], obs_list[-3], obs_list[-2], obs_list[-1]), axis=-1)

        while not done and count < max_steps:
            # [ROS 2 修改] 处理回调函数 (如保持连接活跃或处理其他订阅)
            if node:
                rclpy.spin_once(node, timeout_sec=0)

            if count == 0:
                # 第一步，根据当前状态选择动作 (评估模式)
                action = network.choose_action(np.array(state), np.array(goal[:2]), evaluate=True).clip(-max_action, max_action)
    
                # 将动作转换为线速度和角速度
                a_in = [(action[0] + 1) * linear_cmd_scale, action[1] * angular_cmd_scale]
                last_goal = goal
                # 执行动作
                obs_, _, _, _, _, _ , _, done, goal, target = env.step(a_in, count)        
                observation = np.concatenate((obs_, obs_, obs_, obs_), axis=-1)
                
                for i in range(4):
                    obs_list.append(obs_)           

                if done:
                    print("\n..............................................")
                    print("初始化失败，跳过此回合。")
                    print("..............................................")
                    ep -= 1
                    env.collision -= 1
                    break

                count += 1
                continue
            
            # 选择动作
            act = network.choose_action(np.array(observation), np.array(goal[:2]), evaluate=True).clip(-max_action, max_action)
            a_in = [(act[0] + 1) * linear_cmd_scale, act[1]*angular_cmd_scale]
            
            # 环境步进
            obs_, _, _, _, _, _, reward, done, goal, target = env.step(a_in, count)        
            avg_reward += reward
            
            # 更新观察值堆叠
            observation = np.concatenate((obs_list[-3], obs_list[-2], obs_list[-1], obs_), axis=-1)
            obs_list.append(obs_)
            count += 1
        
        ep += 1
        avg_reward_list.append(avg_reward)
        print("\n..............................................")
        print("%i 轮, 步数: %i, 平均奖励: %f, 碰撞次数: %i " % (ep, count, avg_reward, env.collision))
        print("..............................................")
    
    reward = statistics.mean(avg_reward_list)
    col = env.collision
    print("\n..............................................")
    print("%i 轮评估后的平均奖励, 在 Epoch: %i, 平均奖励: %f, 碰撞次数: %i" % (eval_episodes, epoch, reward, col))
    print("..............................................")
    return reward

def plot_animation_figure():
    """
    绘制训练过程中的各种奖励曲线和动作分布图。
    """
    plt.figure()
    plt.clf()

    # 目标奖励曲线
    plt.subplot(2, 2, 1)
    plt.title(env_name + ' ' + str("SAC") + ' Lr_a: ' + str(lr_a) + ' Lr_c: ' + str(lr_c) +' Target Reward')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.plot(np.arange(ep_real), reward_target_list)

    # 碰撞奖励曲线
    plt.subplot(2, 2, 2)
    plt.title('Collision Reward')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.plot(np.arange(ep_real), reward_collision_list)

    # 油门(线速度)分布
    plt.subplot(2, 2, 3)
    plt.title('Pedal ' + str(ep_real))
    plt.scatter(np.arange(len(pedal_list)), pedal_list, s=6, c='coral')
    
    # 转向(角速度)分布
    plt.subplot(2, 2, 4)
    plt.title('Steering')
    plt.scatter(np.arange(len(steering_list)), steering_list, s=6, c='coral')
    
    plt.tight_layout()

    plt.figure()
    # 总奖励和平均奖励
    plt.subplot(2, 2, 1)
    plt.title(env_name + ' ' + str("SAC") + ' Lr_a: ' + str(lr_a) + ' Lr_c: ' + str(lr_c))
    plt.xlabel('Episode')
    plt.ylabel('Overall Reward')
    plt.plot(np.arange(ep_real), reward_list)
    plt.plot(np.arange(ep_real), reward_mean_list)

    # 启发式奖励
    plt.subplot(2, 2, 2)
    plt.title('Heuristic Reward')
    plt.xlabel('Episode')
    plt.ylabel('Heuristic Reward')
    plt.plot(np.arange(ep_real), reward_heuristic_list)

    # 动作平滑/惩罚奖励
    plt.subplot(2, 2, 3)
    plt.title('Action Reward')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.plot(np.arange(ep_real), reward_action_list)

    # 冻结(停止)奖励
    plt.subplot(2, 2, 4)
    plt.title('Freeze Reward')
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.plot(np.arange(ep_real), reward_freeze_list)

    plt.tight_layout()

    plt.pause(0.001)  # 暂停一小会儿以便更新图表

# 全局变量定义
key_cmd = Twist()
intervention = 0

def key_callback(cmd):
    """
    ROS 订阅者回调函数，用于处理键盘控制指令。
    """
    global key_cmd, intervention
    key_cmd.linear.x = cmd.linear.x
    key_cmd.angular.z = cmd.angular.z
    intervention = cmd.angular.x # 使用 angular.x 作为一个标志位，判断是否有人工干预

if __name__ == "__main__":

    # 设置设备 (GPU 或 CPU)
    device = torch.device("cuda", 0 if torch.cuda.is_available() else "cpu")  # cuda or cpu

    path = os.getcwd()
    yaml_path = os.path.join(path, 'config.yaml')
    # 兼容性处理：如果找不到 config.yaml，尝试在当前文件目录下查找
    if not os.path.exists(yaml_path):
        yaml_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

    with open(yaml_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    ##### 读取每个模型的独立参数 ######
    model = 'GoT-SAC'
    mode_param = config[model]
    model_name = mode_param['name']
    policy_type = mode_param['actor_type']     # 策略网络类型
    critic_type = mode_param['critic_type']    # 价值网络类型
    transformer_block = mode_param['block']    # Transformer 块数量
    transformer_head = mode_param['head']      # Transformer 头数量

    ###### 读取 DRL 默认参数 ######
    max_steps = config['MAX_STEPS']            # 每回合最大步数
    max_episodes = config['MAX_EPISODES']      # 最大训练回合数
    batch_size = config['BATCH_SIZE']          # 批次大小
    lr_a = config['LR_A']                      # Actor 学习率
    lr_c = config['LR_C']                      # Critic 学习率
    gamma = config['GAMMA']                    # 折扣因子
    tau = config['TAU']                        # 软更新系数
    policy_freq = config['ACTOR_FREQ']         # 策略更新频率
    buffer_size = config['BUFFER_SIZE']        # 经验回放缓冲区大小
    frame_stack = config['FRAME_STACK']        # 帧堆叠数量
    plot_interval = config['PLOT_INTERVAL']    # 绘图间隔
    
    ##### 评估参数 #####
    save_interval = config['SAVE_INTERVAL']    # 保存间隔
    save_threshold = config['SAVE_THRESHOLD']  # 保存阈值
    reward_threshold = config['REWARD_THRESHOLD'] # 奖励阈值
    eval_threshold = config['EVAL_THRESHOLD']  # 开始评估的阈值
    eval_ep = config['EVAL_EPOCH']             # 评估的回合数
    save_models = config['SAVE']               # 是否保存模型

    ##### Attention 相关参数 #####
    pre_train = config['PRE_TRAIN']            # 是否使用预训练参数初始化
    attention_only = config['ATTENTION_ONLY']  # 是否仅加载预训练 GoT 的 attention 部分
    policy_attention_fix = config['P_ATTENTION_FIX'] # 是否固定 Policy attention 的权重
    critic_attention_fix = config['C_ATTENTION_FIX'] # 是否固定 Value attention 的权重

    ##### 人工干预参数 #####
    pre_buffer = config['PRE_BUFFER']          # 是否使用人类专家 buffer
    human_guidence = config['HUMAN_INTERVENTION'] # 是否需要人类驾驶员指导

    ##### 熵参数 ######
    auto_tune = config['AUTO_TUNE']            # 是否自动调整熵
    alpha = config['ALPHA']                    # 熵系数
    lr_alpha = config['LR_ALPHA']              # 熵学习率

    ##### 环境参数 ######
    seed = config['SEED']                      # 随机种子
    env_name = config['ENV_NAME']              # 环境名称
    driver = config['DRIVER']                  # 驱动类型
    robot = config['ROBOT']                    # 机器人名称
    linear_cmd_scale = config['L_SCALE']       # 线速度缩放
    angular_cmd_scale = config['A_SCALE']      # 角速度缩放

    # 创建结果存储文件夹
    if not os.path.exists("./results"):
        os.makedirs("./results")
    folder_name = "./final_curves"
    if save_models and not os.path.exists(folder_name):
        os.makedirs(folder_name)
    folder_name = "./final_models"
    if save_models and not os.path.exists(folder_name):
        os.makedirs(folder_name)
        
    # [ROS 2 修改] 初始化 ROS 2
    rclpy.init()
    # 创建 ROS 2 节点
    node = rclpy.create_node('sac_main_node')

    # [ROS 2 修改] 环境初始化
    # ROS 2 不再使用 master_uri，已移除该参数
    # 重要: 你需要同步修改 Environments/env_lab.py 中的 GazeboEnv __init__ 函数，去掉 master_uri 参数
    env = GazeboEnv('main.launch', 1, 1, 1)

    # [ROS 2 修改] 创建订阅者
    # 订阅键盘控制话题，消息类型 Twist，话题名称 /scout/telekey，回调函数 key_callback，队列长度 1 
    cmd_sub = node.create_subscription(Twist, '/scout/telekey', key_callback, 1)
    
    # 原代码的 global 变量初始化保留
    key_cmd = Twist()
    intervention = 0
    
    time.sleep(5) # 等待环境启动

    # 设置随机种子以保证可复现性
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    env.seed(seed)
    
    state, _ = env.reset()
    state_dim = state.shape
    action_dim = 2
    physical_state_dim = 2 # 极坐标
    max_action = 1
    
    # 初始化 SAC 智能体
    ego = SAC(action_dim, physical_state_dim, policy_type, critic_type, policy_attention_fix,
              critic_attention_fix, pre_buffer, seed, lr_c, lr_a, lr_alpha,
              buffer_size, tau, policy_freq, gamma, alpha, block=transformer_block,
              head=transformer_head, automatic_entropy_tuning=auto_tune)

    ###### 如果可能，初始化预训练网络 ######
    if pre_train:
        if attention_only:
            # 仅加载 Attention 部分
            name = "SAC_IL_scout_image_rrc_fisheye_GoT_normalize_Oscar_seed1_64patches_2depth_8heads_2048mlp"
            il_ego = SAC(action_dim, physical_state_dim, policy_type, critic_type, 
                         policy_attention_fix, critic_attention_fix, human_guidence, 
                         seed, lr_c, lr_a, lr_alpha, buffer_size, tau, policy_freq,
                         gamma, alpha, block=transformer_block, head=transformer_head,
                         automatic_entropy_tuning=auto_tune)
            il_ego.load_actor(name, directory="./final_models")
    
            ###### 赋值 Attention 权重 ########
            ego.policy.trans = il_ego.policy.trans
            ego.policy.fc_embed = il_ego.policy.fc_embed

        else:
            # 加载完整 Actor
            name = 'SAC_IL_scout_image_rrc_fisheye_GoT_normalize_Oscar_seed1_64patches_2depth_8heads_2048mlp'
            ego.load_actor(name, directory="./final_models")

    ###### 预初始化专家 Replay Buffer (可选) #######
    if pre_buffer:
        data_dir = '/home/oscar/ws_oscar/DRL-Transformer-SimtoReal-Navigation/catkin_ws/src/gtrl/scripts'
        # 使用 glob 查找数据文件
        files = natsorted(glob.glob(os.path.join(data_dir) + '/IL/Data/' + env_name + '/' + driver + '/*.npz'))
        obs_list = []
        act_list = []
        goal_list = []
        r_list = []
        next_obs_list = []
        next_goal_list = []
        done_list = []
        
        for idx, file in enumerate(files):
            # 读取 .npz 文件数据
            obs = np.load(file)['obs']
            act = np.load(file)['act']
            goal = np.load(file)['goal']
            r = np.load(file)['reward']
            next_obs = np.load(file)['next_obs']
            next_goal = np.load(file)['next_goal']
            done = np.load(file)['done']
            
            obs_list.append(np.array(obs))
            act_list.append(np.array(act))
            goal_list.append(np.array(goal))
            r_list.append(np.array(r))
            next_obs_list.append(np.array(next_obs))
            next_goal_list.append(np.array(next_goal))
            done_list.append(np.array(done))
        
        # 拼接数据
        obs_dataset = np.concatenate(obs_list, axis=0)
        act_dataset = np.concatenate(act_list, axis=0)
        goal_dataset = np.concatenate(goal_list, axis=0)
        reward_dataset = np.concatenate(r_list, axis=0)
        next_obs_dataset = np.concatenate(next_obs_list, axis=0)
        next_goal_dataset = np.concatenate(next_goal_list, axis=0)
        done_dataset = np.concatenate(done_list, axis=0)
    
        # 将数据加载到 SAC 的 expert buffer 中
        ego.initialize_expert_buffer(obs_dataset, act_dataset, goal_dataset[:,:2], 
                                     next_goal_dataset[:,:2], reward_dataset,
                                     next_obs_dataset, done_dataset)

    # 创建评估数据存储
    evaluations = []
    
    ep_real = 0
    done = False
    reward_list = []
    reward_heuristic_list = []
    reward_action_list = []
    reward_freeze_list = []
    reward_target_list = []
    reward_collision_list = []
    reward_mean_list = []
    
    pedal_list = []
    steering_list = []

    plt.ion() # 开启 matplotlib 交互模式

    total_timestep = 0

    try:
        # 开始训练循环
        for ep in tqdm(range(0, max_episodes), ascii=True):
            episode_reward = 0
            episode_heu_reward = 0.0
            episode_act_reward = 0.0
            episode_tar_reward = 0.0
            episode_col_reward = 0.0
            episode_fr_reward = 0.0
            s_list = deque(maxlen=frame_stack)
            s, goal = env.reset()

            for i in range(4):
                s_list.append(s)

            state = np.concatenate((s_list[-4], s_list[-3], s_list[-2], s_list[-1]), axis=-1)

            for timestep in range(max_steps):
                # [ROS 2 修改] 处理 ROS 回调
                # 这是必要的，以便接收来自 /scout/telekey 的键盘指令
                rclpy.spin_once(node, timeout_sec=0)

                # 回合开始时的初始化
                if timestep == 0:
                    action = ego.choose_action(np.array(state), np.array(goal[:2])).clip(-max_action, max_action)
                    a_in = [(action[0] + 1) * linear_cmd_scale, action[1] * angular_cmd_scale]
                    last_goal = goal
                    s_, _, _, _, _, _ , reward, done, goal, target = env.step(a_in, timestep)        
                    state = np.concatenate((s_, s_, s_, s_), axis=-1)
                    
                    for i in range(4):
                        s_list.append(s_)           

                    if done:
                        print("初始化失败，跳过此回合。")
                        break

                    continue
                
                # 回合结束或达到最大步数
                if done or timestep == max_steps-1:
                    ep_real += 1
        
                    done = False

                    # 记录各种奖励
                    reward_list.append(episode_reward)
                    reward_mean_list.append(np.mean(reward_list[-20:]))
                    reward_heuristic_list.append(episode_heu_reward)
                    reward_action_list.append(episode_act_reward)
                    reward_target_list.append(episode_tar_reward)
                    reward_collision_list.append(episode_col_reward)
                    reward_freeze_list.append(episode_fr_reward)

                    # [注释掉的评估代码片段]
                    # if reward_mean_list[-1] >= reward_threshold and ep_real > eval_threshold:
                    #     reward_threshold = reward_mean_list[-1]
                    #     print("Evaluating the Performance.")
                    #     avg_reward = evaluate(ego, env, eval_ep, ep_real, node=node)
                    #     evaluations.append(avg_reward)
                    #     if avg_reward > save_threshold:
                    #         ego.save(file_name, directory=folder_name, reward=int(np.floor(avg_reward)), seed=seed)
                    #         save_threshold = avg_reward

                    pedal_list.clear()
                    steering_list.clear()
                    total_timestep += timestep 
                    print('\n',
                          '\n',
                          'Robot: ', 'Scout',
                          'Episode:', ep_real,
                          'Step:', timestep,
                          'Total Steps:', total_timestep,
                          'R:', episode_reward,
                          'Overall R:', reward_mean_list[-1],
                          'Expert Batch:', np.int8(ego.batch_expert),
                          'Temperature:', ego.alpha.detach().cpu().numpy().item(),
                          'Lr_a:', lr_a,
                          'Lr_c', lr_c,
                          'seed:', seed,
                          'Env:', env_name,
                          "Filename:", model_name,
                          '\n')

                    # 保存奖励曲线数据
                    if (ep_real % save_interval == 0):
                        np.save(os.path.join('final_curves', 'reward_seed' + str(seed) + '_' + model_name),
                                reward_mean_list, allow_pickle=True, fix_imports=True)

                    # 绘图
                    if ep_real % plot_interval == 0:
                        plot_animation_figure()
                        plt.ioff()
                        plt.show()

                    break

                # 处理人工干预
                if intervention:
                    action_exp = [key_cmd.linear.x, key_cmd.angular.z]  # action_expert
                    action = None
                    a_in = [(action_exp[0] + 1) * linear_cmd_scale, action_exp[1]*angular_cmd_scale]
                    pedal_list.append(round((action_exp[0] + 1)/2,2))
                    steering_list.append(round(action_exp[1],2))
                else:
                    # 网络选择动作
                    action = ego.choose_action(np.array(state), np.array(goal[:2])).clip(-max_action, max_action)
                    action_exp = None
                    a_in = [(action[0] + 1) * linear_cmd_scale, action[1]*angular_cmd_scale]
                    # action[0]范围[-1, 1] 
                    # (action[0] + 1) * linear_cmd_scale范围[0, 1]
                    pedal_list.append(round((action[0] + 1)/2,2))  # round 保留两位小数 比如50%
                    steering_list.append(round(action[1],2))

                last_goal = goal
                # 执行动作
                s_, r_h, r_a, r_f, r_c, r_t, reward, done, goal, target = env.step(a_in, timestep)

                # 累加奖励
                episode_reward += reward
                episode_heu_reward += r_h
                episode_act_reward += r_a
                episode_fr_reward += r_f
                episode_col_reward += r_c
                episode_tar_reward += r_t

                next_state = np.concatenate((s_list[-3], s_list[-2], s_list[-1], s_), axis=-1)

                # 将转换 (transition) 存入 Replay Buffer
                ego.store_transition(state, action, last_goal[:2], goal[:2], reward, next_state, intervention, action_exp, done)

                # 训练 SAC 模型
                if human_guidence or pre_buffer:
                    ego.learn_guidence(intervention, batch_size)
                else:
                    ego.learn(batch_size)

                # 更新状态
                state = next_state
                s_list.append(s_)

        # 训练结束后进行评估并保存
        avg_reward = evaluate(ego, env, eval_ep, ep_real, node=node)
        evaluations.append(avg_reward)
        if avg_reward > save_threshold:
            ego.save(model_name, directory=folder_name, reward=int(np.floor(avg_reward)), seed=seed)

        np.save(os.path.join('final_curves', 'reward_seed' + str(seed) + '_' + model_name), reward_mean_list, allow_pickle=True, fix_imports=True)

    finally:
        # [ROS 2 修改] 程序退出时关闭 ROS 2 节点
        print("Shutting down ROS 2 node...")
        rclpy.shutdown()