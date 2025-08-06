#!/bin/bash

# 定义远程路径、本地路径和要排除的文件夹
remote_path1="user@10.21.41.1:/home/user/rl_deploy_wt/rl_deploy_for_Cr1/"
local_path1="/home/tian/Desktop/learn/imation/deploy/rl_deploy_for_Cr1/"
exclude_folders1=(".vscode" 
                  "logs" 
                  ".git"
                  "output"
                  "build"
                  ) 
 

# 构建排除参数
exclude_args=""
for folder in "${exclude_folders1[@]}"; do
    exclude_args="$exclude_args --exclude=$folder"
done

# 执行 rsync 命令
rsync -avz $exclude_args $local_path1 $remote_path1
# ssh steve@s4.v100.vip -p 25168
# if [ $? -eq 0 ]; then
#     # 定义要在远程执行的命令
#     remote_commands='
# cd ~/project/lab_ws/LeggedLab
# nohup python legged_lab/scripts/train.py --task=pi_flat --num_envs=4096 --headless --device=cuda:3 > /dev/null 2>&1 &
# '
#     # 使用 SSH 执行远程命令
#     ssh steve@192.168.21.99 "$remote_commands"
# else
#     echo "rsync 同步失败，未执行远程命令。"
# fi