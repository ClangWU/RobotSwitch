#!/usr/bin/python

#@markdown ### **Imports**
# diffusion policy import
from typing import Tuple, Sequence, Dict, Union, Optional
import numpy as np
import math
import torch
import torch.nn as nn
import collections
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler
from diffusers.training_utils import EMAModel
from diffusers.optimization import get_scheduler
from tqdm.auto import tqdm
# from algorithm.dp.noisenet import ConditionalUnet1D
# # env import
# import algorithm.dp.dataset as ds
from huggingface_hub.utils import IGNORE_GIT_FOLDER_PATTERNS
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32MultiArray
import time
from argparse import ArgumentParser
from constants import HANDLOAD, START_ARM_POSE, END_ARM_POSE
from geometry_msgs.msg import PoseStamped
import threading
msg_pose_lock = threading.Lock()
msg_pose = []
def callback_function(msg):  
    global msg_pose       
    msg_pose = msg.data

def message_thread():
    obs_sub = rospy.Subscriber('/observation', Float32MultiArray, callback_function)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('real_env_node', anonymous=True)
    action_pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=10)
    
    msg_thread = threading.Thread(target=message_thread)
    msg_thread.start()
    time.sleep(1)
    load_pretrained = True
    n_games = 20
    pred_horizon = 16
    obs_horizon = 2
    # action_horizon = 8
    # batch_size = 256
    # n_epochs = 100
    # alpha = 0.001
    # scores_list = []
    # actions_path = "../doc/actions.csv"
    # states_path = "../doc/observations.csv"
    # episode_ends_path = "../doc/episode_ends.csv"
    # dataset = ds.FrankaDataset(
    #     actions_path, states_path, episode_ends_path,
    #     pred_horizon, obs_horizon, action_horizon)
    # # save training data statistics (min, max) for each dim 
    # stats = dataset.stats
    # # create dataloader
    # dataloader = torch.utils.data.DataLoader(
    #     dataset,
    #     batch_size=batch_size,
    #     num_workers=1,
    #     shuffle=True,
    #     pin_memory=True,
    #     persistent_workers=True)
    # # visualize data in batch
    # batch = next(iter(dataloader))
    # print("batch['obs'].shape:", batch['obs'].shape)
    # print("batch['action'].shape", batch['action'].shape)
    # # observation and action dimensions corrsponding to
    # obs_dim = 10
    # action_dim = 7
    # # noise prediction network
    # noise_pred_net = ConditionalUnet1D(
    #     input_dim=action_dim,
    #     global_cond_dim=obs_dim*obs_horizon)
    # # example inputs
    # # 生成带噪声的动作
    # noised_action = torch.randn((1, pred_horizon, action_dim))
    # # 生成观测
    # obs = torch.zeros((1, obs_horizon, obs_dim))
    # diffusion_iter = torch.zeros((1,))
    # # 网络对带噪声的动作进行预测，生成噪声
    # noise = noise_pred_net(
    #     sample=noised_action,
    #     timestep=diffusion_iter,
    #     global_cond=obs.flatten(start_dim=1))
    # # 生成去噪声动作
    # denoised_action = noised_action - noise
    # # for this demo, we use DDPMScheduler with 100 diffusion iterations
    # num_diffusion_iters = 100
    # noise_scheduler = DDPMScheduler(
    #     num_train_timesteps=num_diffusion_iters,
    #     beta_schedule='squaredcos_cap_v2',
    #     clip_sample=True,
    #     prediction_type='epsilon'
    # )
    # device = torch.device('cuda')
    # _ = noise_pred_net.to(device)
    # num_epochs = 100
    # # 初始化一个 EMA 模型，用于跟踪网络权重的移动平均。这有助于稳定训练过程并加速收敛。
    # ema = EMAModel(
    #     parameters=noise_pred_net.parameters(),
    #     power=0.75)
    # if load_pretrained:
    #   ckpt_path = "./tmp/realrobot/cutting_noise_pred_net.pt"
    #   state_dict = torch.load(ckpt_path, map_location='cuda')
    #   ema_noise_pred_net = noise_pred_net
    #   ema_noise_pred_net.load_state_dict(state_dict)
    #   print('Pretrained weights loaded.')
    # else:
    #   print("Skipped pretrained weight loading.")
    # #@markdown ### **Inference**
    # # limit enviornment interaction to 200 steps before termination
    # max_steps = 200
    # # use a seed > 200 to avoid initial states seen in the training dataset

    for id in range(n_games):
    # while True:
    #     # get first observation
    #     obs, info = env.reset()
    #     # keep a queue of last 2 steps of observations
        print('msg_pose', msg_pose)
        obs_deque = collections.deque(
            [msg_pose] * obs_horizon, maxlen=obs_horizon)
        time.sleep(0.002)
        msg  = PoseStamped()
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 0
        msg.pose.orientation.x = 1
        msg.pose.orientation.y = 0
        msg.pose.orientation.z = 0
        action_pub.publish(msg)

    #     # save visualization and rewards
    #     rewards = list()
    #     done = False
    #     step_idx = 0
    #     score = 0
    #     with tqdm(total=max_steps, desc="Franka Cutting") as pbar:
    #         while not done:
    #             B = 1
    #             # stack the last obs_horizon (2) number of observations
    #             obs_seq = np.stack(obs_deque)
    #             # normalize observation
    #             nobs = ds.normalize_data(obs_seq, stats=stats['obs'])
    #             # device transfer
    #             nobs = torch.from_numpy(nobs).to(device, dtype=torch.float32)
    #             # infer action
    #             with torch.no_grad():
    #                 # reshape observation to (B,obs_horizon*obs_dim)
    #                 obs_cond = nobs.unsqueeze(0).flatten(start_dim=1)
    #                 # initialize action from Guassian noise
    #                 noisy_action = torch.randn(
    #                     (B, pred_horizon, action_dim), device=device)
    #                 naction = noisy_action
    #                 # init scheduler
    #                 noise_scheduler.set_timesteps(num_diffusion_iters)
    #                 for k in noise_scheduler.timesteps:
    #                     # predict noise
    #                     noise_pred = ema_noise_pred_net(
    #                         sample=naction,
    #                         timestep=k,
    #                         global_cond=obs_cond
    #                     )
    #                     # inverse diffusion step (remove noise)
    #                     naction = noise_scheduler.step(
    #                         model_output=noise_pred,
    #                         timestep=k,
    #                         sample=naction
    #                     ).prev_sample
    #             # unnormalize action
    #             naction = naction.detach().to('cpu').numpy()
    #             # (B, pred_horizon, action_dim)
    #             naction = naction[0]
    #             # 将归一化的动作转换为原始动作
    #             action_pred = ds.unnormalize_data(naction, stats=stats['action'])
    #             # only take action_horizon number of actions
    #             # 看着论文就知道下面的顺序是怎么来的了
    #             start = obs_horizon - 1
    #             end = start + action_horizon
    #             action = action_pred[start:end,:]
    #             # 执行8个动作，不需要重新规划
    #             # without replanning
    #             for i in range(len(action)):
    #                 # print('action', action[i], 'step', step_idx, 'i', i)
    #                 # stepping env
    #                 observation, reward, terminated, truncated, info = env.step(action[i])
    #                 obs = observation["qpos"].tolist() + observation["force"][0:3].tolist() + observation["cut_height"][0:3].tolist()
    #                 # save observations
    #                 obs_deque.append(obs)
    #                 # and reward/vis
    #                 score += reward
    #                 rewards.append(reward)
    #                 # update progress bar
    #                 step_idx += 1
    #                 pbar.update(1)
    #                 pbar.set_postfix(reward=reward)
    #                 if terminated:
    #                     env.auto_finish()
    #                     break
    #                 if truncated:
    #                     break
    #     scores_list.append(score)
    #     # print out the score
    #     print('Game:', id+1, 'Score:', score)
    print('msg_pose', msg_pose)

    # games = list(range(1, n_games + 1))

    # # 绘制得分曲线图
    # plt.figure(figsize=(10, 6))
    # plt.plot(games, scores_list, marker='o', color='b')
    # plt.title('Scores over Games')
    # plt.xlabel('Game Number')
    # plt.ylabel('Score')
    # plt.grid(True)
    # plt.xticks(games)
    # plt.savefig('./plots/realrobot/realrobot.png')
    # plt.show()

