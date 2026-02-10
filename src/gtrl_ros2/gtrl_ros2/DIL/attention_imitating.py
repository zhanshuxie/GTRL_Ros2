#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul  9 18:31:19 2022

@author: oscar
"""
import sys
import os
import glob
import random
import numpy as np
from tqdm import tqdm
import matplotlib.pyplot as plt
from natsort import natsorted

import torch
import torch.optim as optim
from torch.utils.data import Dataset
from torch.utils.data import DataLoader
from torch.utils.data import random_split

# [ROS 2 修改] 修正导入路径，使用包内导入或绝对包路径
# 假设 gtrl_ros2 是你的包名
try:
    from gtrl_ros2.SAC.DRL import SAC
except ImportError:
    # 如果作为脚本直接运行且路径未设置，尝试相对路径回退（开发调试用）
    sys.path.append(os.path.join(os.path.dirname(__file__), '../../'))
    from gtrl_ros2.SAC.DRL import SAC

if torch.cuda.is_available():
    device = torch.device("cuda")
else:
    device = torch.device("cpu")
print('Use:', device)

class DatasetWrapper(Dataset):
    def __init__(self, data1, data2, data3):
        self.data1 = data1
        self.data2 = data2
        self.data3 = data3

    def __len__(self):
        return len(self.data1)

    def __getitem__(self, index):
        x1 = self.data1[index]
        x2 = self.data2[index]
        x3 = self.data3[index]
        return x1, x2, x3

def train(epoch):

    train_loss = 0.0

    for i, (obs_data, act_data, goal_data) in enumerate(train_loader):
        observation = obs_data.float().to(device)
        action = act_data.float().to(device)
        goal = goal_data.float().to(device)

        observation = observation.float().permute(0,3,1,2).to(device)
        goal = goal[:, :2]

        predict, log_prob, mean = ego.policy.sample([observation, goal])

        mean = mean.clip(-max_action, max_action)
        optimizer.zero_grad()
        loss = torch.sqrt(torch.pow(mean - action, 2).mean())
        loss.backward()
        torch.nn.utils.clip_grad_norm_(ego.policy.parameters(), 10)
        optimizer.step()
        train_loss += loss.detach().cpu().numpy().mean()

    return round(train_loss/(i+1), 4)

def val(epoch):
    val_loss = 0.0

    with torch.no_grad():
        for i, (obs_data, act_data, goal_data) in enumerate(val_loader):
            observation = obs_data.float().to(device)
            action = act_data.float().to(device)
            goal = goal_data.float().to(device)
        
            observation = observation.float().permute(0,3,1,2).to(device)
            goal = goal[:, :2]
        
            predict, log_prob, mean = ego.policy.sample([observation, goal])
        
            mean = mean.clip(-max_action, max_action)
            loss = torch.sqrt(torch.pow(mean - action, 2).mean())
            val_loss += loss.detach().cpu().numpy().mean()

    return round(val_loss/(i+1), 4)

if __name__ == "__main__":

    file_path = os.getcwd()
    env_name = 'RRC' # 变量名统一
    driver = 'Oscar_GoT_augmentend'
    
    # 路径处理
    data_dir = os.path.join(file_path, 'Data', env_name, driver)
    if not os.path.exists(data_dir):
        # 尝试使用相对路径寻找数据（假设在 src/gtrl_ros2/scripts 下）
        # 这里需要用户确认数据存放的确切位置，ROS 2 中通常不在 install 目录放数据
        pass

    files = natsorted(glob.glob(data_dir + '/*.npz'))
    
    seed = 1
    iteration = 600
    batch_size = 32
    lr_a = 1e-3
    lr_c = 1e-3
    lr_il = 1e-3
    lr_alpha = 1e-4
    gamma = 0.999
    tau = 0.005
    buffer_size = 10
    file_name = "gtrl"
    frame_stack = 4
    plot_interval = int(1)
    policy_type = "GaussianTransformer"
    critic_type = "CNN"
    policy_attention_fix = False
    critic_attention_fix = False
    pre_buffer = False
    alpha = 1.0
    auto_tune = False
    action_dim = 2
    max_action = 1
    policy_freq = 1
    physical_state_dim = 2

    torch.manual_seed(seed)
    torch.cuda.manual_seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False

    obs_list = []
    act_list = []
    goal_list = []
    
    if not files:
        print(f"No data files found in {data_dir}. Please check path.")
        sys.exit(1)

    ######## Read the Dataset ###########
    print("Loading data...")
    for idx, file in enumerate(files):
        
        obs = np.load(file)['obs']
        act = np.load(file)['act']
        goal = np.load(file)['goal']
        
        obs_list.append(np.array(obs))
        act_list.append(np.array(act))
        goal_list.append(np.array(goal))
    
    ######### Split the dataset #########    
    obs_dataset = np.concatenate(obs_list, axis=0)
    obs_train_size = int(0.8*len(obs_dataset))
    obs_val_size = len(obs_dataset) - obs_train_size
    obs_train_set, obs_val_set = random_split(obs_dataset, [obs_train_size, obs_val_size])
    obs_train_idx = obs_train_set.indices
    obs_val_idx = obs_val_set.indices
    
    act_dataset = np.concatenate(act_list, axis=0)
    goal_dataset = np.concatenate(goal_list, axis=0)
    
    ######### Wrap to dataloader ########
    obs_train_sample = obs_dataset[obs_train_idx]
    obs_val_sample = obs_dataset[obs_val_idx]

    act_train_sample = act_dataset[obs_train_idx]
    act_val_sample = act_dataset[obs_val_idx]

    goal_train_sample = goal_dataset[obs_train_idx]
    goal_val_sample = goal_dataset[obs_val_idx]
    
    train_ensemble = DatasetWrapper(obs_train_sample, act_train_sample, goal_train_sample)
    val_ensemble = DatasetWrapper(obs_val_sample, act_val_sample, goal_val_sample)
    train_loader = \
        DataLoader(train_ensemble, batch_size=batch_size, shuffle=True, num_workers=4)
    val_loader = \
        DataLoader(val_ensemble, batch_size=batch_size, shuffle=True, num_workers=4)

    ######### Initialize DRL agent #######
    # 注意：auto_tune 参数在原来的 SAC 定义中是 automatic_entropy_tuning，这里可能需要根据 SAC 定义确认
    ego = SAC(action_dim, physical_state_dim, policy_type, critic_type, policy_attention_fix,
              critic_attention_fix, pre_buffer, seed, lr_c, lr_a, lr_alpha,
              buffer_size, tau, policy_freq, gamma, alpha, automatic_entropy_tuning=auto_tune)

    optimizer = optim.Adam(ego.policy.parameters(), lr=lr_il)

    ######### Trainining ########
    fig = plt.figure()
    ax = plt.subplot()
    min_val_loss = 10
    val_low_idx = 0
    train_loss_list = []
    val_loss_list = []
    
    model_save_dir = "./pytorch_models"
    if not os.path.exists(model_save_dir):
        os.makedirs(model_save_dir)

    for epoch in tqdm(range(0, iteration), ascii=True):
        train_loss_epoch = train(epoch)
        val_loss_epoch = val(epoch)

        train_loss_list.append(train_loss_epoch)
        val_loss_list.append(val_loss_epoch)
        
        print('Epoch:%i, Train and Validation loss are:%f, %f' % (epoch, train_loss_epoch, val_loss_epoch))

        if val_loss_list[-1] < min_val_loss:
            val_low_idx = epoch
            torch.save(ego.policy.state_dict(), '%s/%s_actor.pth' % (model_save_dir, file_name))
            # torch.save(ego.policy, '%s/%s_actor_model.pth' % (model_save_dir, file_name)) # 通常建议只保存 state_dict
            min_val_loss = val_loss_list[-1]

        if (int(epoch) + 1 == iteration):
            ax.scatter(val_low_idx, min_val_loss, marker='*', s=128, color='cornflowerblue', label='Lowest Validation Loss Epoch')

        if (int(epoch) + 1 == iteration):
            ax.plot(np.arange(len(train_loss_list)), train_loss_list, label='Train Loss', color='lightseagreen')
            ax.plot(val_loss_list, label='Validation Loss', color='tomato')
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.set_xlabel('Epoch')
            ax.set_ylabel('RMSE Loss')
            ax.legend(frameon=False)
            plt.show()