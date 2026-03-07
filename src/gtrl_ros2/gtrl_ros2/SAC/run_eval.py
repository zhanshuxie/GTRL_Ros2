#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import time
import yaml
import statistics
import numpy as np
from collections import deque

import torch
import rclpy
from rclpy.node import Node

# 导入自定义的 SAC 算法和 Gazebo 环境
from gtrl_ros2.SAC.DRL import SAC
from gtrl_ros2.Environments.env_lab import GazeboEnv

def evaluate_model(network, env, eval_episodes, max_steps, frame_stack, max_action, linear_cmd_scale, angular_cmd_scale, node=None):
    """
    独立的评估函数
    """
    obs_list = deque(maxlen=frame_stack)
    env.collision = 0
    ep = 0
    avg_reward_list = []
    
    while ep < eval_episodes:
        count = 0
        obs, goal = env.reset() 
        done = False
        avg_reward = 0.0

        # 初始化帧堆叠
        for i in range(4):
            obs_list.append(obs)

        observation = np.concatenate((obs_list[-4], obs_list[-3], obs_list[-2], obs_list[-1]), axis=-1)

        while not done and count < max_steps:
            if node:
                rclpy.spin_once(node, timeout_sec=0)

            # 根据当前状态选择动作 (evaluate=True 意味着取策略的均值动作，无随机性)
            act = network.choose_action(np.array(observation), np.array(goal[:2]), evaluate=True).clip(-max_action, max_action)
            a_in = [(act[0] + 1) * linear_cmd_scale, act[1] * angular_cmd_scale]
            
            # 环境步进
            obs_, r_h, r_a, r_f, r_c, r_t, reward, done, goal, target = env.step(a_in, count)        
            avg_reward += reward
            
            if count == 0 and done:
                print("\n..............................................")
                print("初始化失败，跳过此回合。")
                print("..............................................")
                # 如果第一步就结束了，视为无效回合，不增加轮数
                env.collision -= 1
                break

            # 更新观察值堆叠
            observation = np.concatenate((obs_list[-3], obs_list[-2], obs_list[-1], obs_), axis=-1)
            obs_list.append(obs_)
            count += 1
        
        # 只有正常结束或达到最大步数的回合才被计入
        if count > 0 and (done or count == max_steps):
            ep += 1
            avg_reward_list.append(avg_reward)
            print("\n..............................................")
            print("第 %i 轮评估完成, 存活步数: %i, 奖励: %f, 累计碰撞次数: %i " % (ep, count, avg_reward, env.collision))
            print("..............................................")
    
    final_reward = statistics.mean(avg_reward_list)
    final_col = env.collision
    print("\n==============================================")
    print(" 最终评估报告 ")
    print(" 共评估 %i 轮, 平均奖励: %f, 总碰撞次数: %i" % (eval_episodes, final_reward, final_col))
    print("==============================================\n")
    return final_reward

def main():
    # 初始化 ROS 2
    rclpy.init()
    node = rclpy.create_node('sac_eval_node')

    # 读取配置文件
    path = os.getcwd()
    yaml_path = os.path.join(path, 'config.yaml')
    if not os.path.exists(yaml_path):
        yaml_path = os.path.join(os.path.dirname(__file__), 'smoke_test_config.yaml')

    with open(yaml_path) as f:
        config = yaml.load(f, Loader=yaml.FullLoader)

    ##### 读取参数 ######
    model = 'GoT-SAC'
    mode_param = config[model]
    policy_type = mode_param['actor_type']     
    critic_type = mode_param['critic_type']    
    transformer_block = mode_param['block']    
    transformer_head = mode_param['head']      

    max_steps = config['MAX_STEPS']            
    lr_a = config['LR_A']                      
    lr_c = config['LR_C']                      
    gamma = config['GAMMA']                    
    tau = config['TAU']                        
    policy_freq = config['ACTOR_FREQ']         
    buffer_size = config['BUFFER_SIZE']        
    frame_stack = config['FRAME_STACK']        
    
    policy_attention_fix = config['P_ATTENTION_FIX'] 
    critic_attention_fix = config['C_ATTENTION_FIX'] 
    pre_buffer = config['PRE_BUFFER']          
    auto_tune = config['AUTO_TUNE']            
    alpha = config['ALPHA']                    
    lr_alpha = config['LR_ALPHA']              

    seed = config['SEED']                      
    linear_cmd_scale = config['L_SCALE']       
    angular_cmd_scale = config['A_SCALE']      
    
    # 评估专用参数
    eval_episodes = 15  # 你可以在这里修改评估的轮数

    # 环境初始化
    print("正在启动 Gazebo 环境...")
    env = GazeboEnv('simulation.launch.py', 1, 1, 1)
    time.sleep(5) # 等待环境启动完成

    # 固定随机种子
    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    np.random.seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    env.seed(seed)
    
    action_dim = 2
    physical_state_dim = 2 
    max_action = 1
    
    # 初始化 SAC 智能体
    print("正在初始化 SAC 智能体...")
    ego = SAC(action_dim, physical_state_dim, policy_type, critic_type, policy_attention_fix,
              critic_attention_fix, pre_buffer, seed, lr_c, lr_a, lr_alpha,
              buffer_size, tau, policy_freq, gamma, alpha, block=transformer_block,
              head=transformer_head, automatic_entropy_tuning=auto_tune)

    # ================= 修改这里的文件名 =================
    model_filename = 'gtrl_reward89_seed525'
    model_dir = './final_models'
    # ====================================================

    print(f"尝试加载模型权重: {model_dir}/{model_filename}_actor.pth ...")
    try:
        # 注意：load_actor 内部会自动加上 _actor.pth，所以这里只传前缀
        ego.load_actor(model_filename, directory=model_dir)
        print(">>> 模型加载成功！准备开始评估 <<<")
    except Exception as e:
        print(f"模型加载失败，错误信息: {e}")
        print("请检查 final_models 文件夹下是否存在该文件，以及 config.yaml 的网络结构是否匹配。")
        rclpy.shutdown()
        sys.exit()

    # 开始评估
    evaluate_model(ego, env, eval_episodes, max_steps, frame_stack, max_action, linear_cmd_scale, angular_cmd_scale, node)

    # 结束清理
    print("评估结束，正在关闭 ROS 2 节点...")
    rclpy.shutdown()

if __name__ == '__main__':
    main()