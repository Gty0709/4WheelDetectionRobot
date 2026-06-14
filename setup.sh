#!/bin/bash
# ============================================================
# mickrobot 模块A+B+C 一键环境配置脚本
# 适用：Ubuntu 24.04 + ROS2 Jazzy
# 用法：bash setup.sh
# ============================================================
set -e

ROS_DISTRO=jazzy
WS_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "================================================"
echo " mickrobot 模块A+B+C 环境配置"
echo " 工作空间：$WS_DIR"
echo " ROS 发行版：$ROS_DISTRO"
echo "================================================"

# ── 1. 检查 ROS2 环境 ────────────────────────────────────────
if [ ! -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    echo "[错误] 未找到 /opt/ros/$ROS_DISTRO/setup.bash"
    echo "  请先安装 ROS2 Jazzy："
    echo "  wget http://fishros.com/install -O fishros && bash fishros"
    exit 1
fi
source /opt/ros/$ROS_DISTRO/setup.bash
echo "[OK] ROS2 $ROS_DISTRO 已加载"

# ── 2. 安装模块A依赖 ─────────────────────────────────────────
echo ""
echo "[步骤1] 安装模块A（机器人描述 + Gazebo 仿真）依赖..."
sudo apt-get update -q
sudo apt-get install -y \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-teleop-twist-keyboard
echo "[OK] 模块A依赖安装完成"

# ── 3. 安装模块B依赖 ─────────────────────────────────────────
echo ""
echo "[步骤2] 安装模块B（SLAM + 定位）依赖..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-nav2-amcl \
    ros-$ROS_DISTRO-nav2-map-server \
    ros-$ROS_DISTRO-nav2-lifecycle-manager \
    ros-$ROS_DISTRO-nav2-rviz-plugins \
    ros-$ROS_DISTRO-topic-tools \
    ros-$ROS_DISTRO-tf2-tools \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-cartographer-ros
echo "[OK] 模块B依赖安装完成"

# ── 4. 安装模块C依赖 ─────────────────────────────────────────
echo ""
echo "[步骤3] 安装模块C（Nav2 导航）依赖..."
sudo apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-nav2-simple-commander
echo "[OK] 模块C依赖安装完成"

# ── 5. 确认 src/ 包目录存在 ────────────────────────────────────────
echo ""
echo "[步骤4] 检查 src/ 包..."
for pkg in mickrobot_description perception_pkg navigation_pkg; do
    if [ ! -f "$WS_DIR/src/$pkg/package.xml" ]; then
        echo "[错误] 缺少 src/$pkg/package.xml"
        exit 1
    fi
    echo "  [OK] src/$pkg"
done
echo "[OK] 包目录就绪"

# ── 6. colcon build ──────────────────────────────────────────
echo ""
echo "[步骤5] 编译工作空间..."
cd "$WS_DIR"
colcon build --symlink-install --base-paths src 2>&1
echo "[OK] 编译完成"

# ── 7. 写入 ~/.bashrc ────────────────────────────────────────
echo ""
echo "[步骤6] 配置 ~/.bashrc 自动 source..."
BASHRC_LINE="source $WS_DIR/install/setup.bash"
if ! grep -qF "$BASHRC_LINE" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# mickrobot workspace" >> ~/.bashrc
    echo "$BASHRC_LINE" >> ~/.bashrc
    echo "  已添加到 ~/.bashrc"
else
    echo "  ~/.bashrc 已包含，跳过"
fi

echo ""
echo "================================================"
echo " 配置完成！请执行："
echo "   source ~/.bashrc"
echo " 然后参考以下文档："
echo "   src/mickrobot_description/开发文档.md（模块A）"
echo "   src/perception_pkg/开发文档.md    （模块B）"
echo "   src/navigation_pkg/开发文档_C.md  （模块C）"
echo "================================================"
