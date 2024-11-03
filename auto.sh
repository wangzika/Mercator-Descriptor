#!/bin/bash

# 定义序列编号的数组
seq_idxs=("00" "02" "05" "08") # 这里添加你需要的序列编号
# seq_idxs=("06" "07") # 这里添加你需要的序列编号


# 循环遍历序列编号数组
for seq_idx in "${seq_idxs[@]}"
do
  echo "Launching for sequence index: $seq_idx"
  # 调用roslaunch，设置seq_idx的值
  roslaunch std_detector demo_scancontext.launch seq_idx:=$seq_idx
  # roslaunch std_detector demo_kitti.launch seq_idx:=$seq_idx

  # 等待一段时间或者某个条件，确保当前实例结束后再启动下一个demo_kitti.launch
  # sleep 10s # 根据需要取消注释此行并调整等待时间
done
   