#!/bin/sh

## fordebug
# echo "$0"
# echo "$(dirname "$0")"
## 获取脚本所在目录（兼容 source 和直接执行）
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

## shell/ → pcl_detection/ → src/ → workspace/
WORKSPACE="$(dirname "$(dirname "$(dirname "$SCRIPT_DIR")")")"

## source 工作空间环境
. "$WORKSPACE/devel/setup.${SHELL##*/}"

## 启动 launch 文件
roslaunch pcl_detection2 pcl_detection.launch
