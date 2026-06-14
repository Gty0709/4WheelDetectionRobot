#!/bin/bash
# [Humble迁移] Gazebo Classic + RViz GPU 渲染（NVIDIA，默认启用）
# 用法: source scripts/gazebo_gpu_env.sh
# 注意: 勿设置 LIBGL_ALWAYS_SOFTWARE=1，否则会强制 CPU 软渲染

export LIBGL_ALWAYS_SOFTWARE=0
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
export __VK_LAYER_NV_optimus=NVIDIA_only
# 双 NVIDIA 时优先独显（可按本机 nvidia-smi 顺序调整）
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-0}"
# 勿覆盖 GAZEBO_RESOURCE_PATH / GAZEBO_MODEL_PATH，由 bringup_classic.launch.py 与系统 setup.sh 管理
