#!/usr/bin/env python3
"""
依次在 4 个终端窗口启动手动建图流程（间隔默认 2s）。

对应 README「手动分终端建图」：
  1. humble_sim_slam  2. rviz_slam  3. teleop_slider  4. detection

用法（仓库根目录）:
  python3 scripts/launch_mapping_terminals.py
  python3 scripts/launch_mapping_terminals.py --delay 3 --kill-first

指定终端模拟器:
  TERMINAL=gnome-terminal python3 scripts/launch_mapping_terminals.py
"""

from __future__ import annotations

import argparse
import os
import shlex
import shutil
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ENV_SCRIPT = ROOT / 'env_humble.bash'
KILL_SCRIPT = ROOT / 'scripts' / 'kill_sim.sh'

STEPS = [
    (
        '1-SLAM',
        'ros2 launch perception_pkg humble_sim_slam.launch.py',
    ),
    (
        '2-RViz',
        'ros2 launch perception_pkg rviz_slam.launch.py',
    ),
    (
        '3-Teleop',
        'ros2 launch perception_pkg teleop_slider.launch.py',
    ),
    (
        '4-Detection',
        'ros2 launch detection_pkg detection.launch.py',
    ),
]


def _shell_command(launch_cmd: str) -> str:
    """在子终端里 cd 到仓库、加载环境并执行 launch。"""
    return (
        f'cd "{ROOT}" && '
        f'source "{ENV_SCRIPT}" && '
        f'{launch_cmd}; '
        f'echo; echo "[{launch_cmd}] 已退出 (code=$?)"; '
        f'exec bash'
    )


def _detect_terminal() -> str | None:
    explicit = os.environ.get('TERMINAL', '').strip()
    if explicit:
        return explicit if shutil.which(explicit) else None
    for name in (
        'gnome-terminal',
        'konsole',
        'xfce4-terminal',
        'mate-terminal',
        'x-terminal-emulator',
        'xterm',
    ):
        if shutil.which(name):
            return name
    return None


def _open_terminal(terminal: str, title: str, command: str) -> subprocess.Popen:
    shell_cmd = _shell_command(command)
    if terminal == 'gnome-terminal':
        return subprocess.Popen(
            [terminal, f'--title={title}', '--', 'bash', '-lc', shell_cmd],
            cwd=str(ROOT),
        )
    if terminal == 'konsole':
        return subprocess.Popen(
            [terminal, '--new-tab', '-p', f'tabtitle={title}', '-e', 'bash', '-lc', shell_cmd],
            cwd=str(ROOT),
        )
    if terminal in ('xfce4-terminal', 'mate-terminal'):
        return subprocess.Popen(
            [
                terminal, f'--title={title}', '--hold',
                '-e', f'bash -lc {shlex.quote(shell_cmd)}',
            ],
            cwd=str(ROOT),
        )
    if terminal == 'xterm':
        return subprocess.Popen(
            [terminal, '-T', title, '-e', 'bash', '-lc', shell_cmd],
            cwd=str(ROOT),
        )
    # x-terminal-emulator 等 Debian 通用入口
    return subprocess.Popen(
        [terminal, '-e', 'bash', '-lc', shell_cmd],
        cwd=str(ROOT),
    )


def _open_tmux(steps: list[tuple[str, str]], delay: float) -> None:
    if not shutil.which('tmux'):
        print('[launch_mapping] 未找到图形终端，也未安装 tmux。', file=sys.stderr)
        sys.exit(1)
    session = 'robothomework_mapping'
    subprocess.run(['tmux', 'kill-session', '-t', session], check=False, stdout=subprocess.DEVNULL)
    for idx, (title, cmd) in enumerate(steps):
        if idx > 0:
            time.sleep(delay)
        shell_cmd = _shell_command(cmd)
        if idx == 0:
            subprocess.check_call(
                ['tmux', 'new-session', '-d', '-s', session, '-n', title, 'bash', '-lc', shell_cmd],
            )
        else:
            subprocess.check_call(
                ['tmux', 'new-window', '-t', session, '-n', title, 'bash', '-lc', shell_cmd],
            )
        print(f'[launch_mapping] tmux 窗口 {idx + 1}/4: {title}')
    subprocess.check_call(['tmux', 'select-window', '-t', f'{session}:0'])
    print(f'[launch_mapping] 附加会话: tmux attach -t {session}')
    subprocess.check_call(['tmux', 'attach', '-t', session])


def main() -> int:
    parser = argparse.ArgumentParser(description='分 4 个终端依次启动建图流程')
    parser.add_argument(
        '--delay', type=float, default=2.0,
        help='相邻两个终端启动间隔（秒），默认 2',
    )
    parser.add_argument(
        '--kill-first', action='store_true',
        help='启动前先 bash scripts/kill_sim.sh 清理僵尸 Gazebo',
    )
    parser.add_argument(
        '--tmux', action='store_true',
        help='强制使用 tmux（4 个窗口）而非图形终端',
    )
    args = parser.parse_args()

    if not ENV_SCRIPT.is_file():
        print(f'[launch_mapping] 缺少 {ENV_SCRIPT}', file=sys.stderr)
        return 1

    if args.kill_first:
        if KILL_SCRIPT.is_file():
            print('[launch_mapping] 清理旧仿真与遥控终端进程…')
            subprocess.run(['bash', str(KILL_SCRIPT)], cwd=str(ROOT), check=False)
        else:
            print('[launch_mapping] 警告: 未找到 kill_sim.sh', file=sys.stderr)

    if args.tmux:
        _open_tmux(STEPS, args.delay)
        return 0

    terminal = _detect_terminal()
    if terminal is None:
        print('[launch_mapping] 未检测到 gnome-terminal/konsole 等，改用 tmux…')
        _open_tmux(STEPS, args.delay)
        return 0

    print(f'[launch_mapping] 使用终端: {terminal}，间隔 {args.delay}s')
    for idx, (title, cmd) in enumerate(STEPS):
        if idx > 0:
            time.sleep(args.delay)
        _open_terminal(terminal, title, cmd)
        print(f'[launch_mapping] 已启动 {idx + 1}/4: {title} → {cmd}')

    print('[launch_mapping] 全部已拉起。结束建图建议顺序: 终端4 → 终端1 → 2/3')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
