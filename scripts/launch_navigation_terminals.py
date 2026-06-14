#!/usr/bin/env python3
"""
依次在 3 个终端窗口启动航点巡逻导航流程（间隔默认 4s）。

  1. Gazebo 仿真
  2. AMCL 定位 + Nav2
  3. RViz + 航点巡逻节点

用法（仓库根目录）:
  python3 scripts/launch_navigation_terminals.py
  python3 scripts/launch_navigation_terminals.py --kill-first --delay 5
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
SESSION = 'src/perception_pkg/maps/map_latest'

STEPS = [
    (
        '1-Sim',
        (
            'ros2 launch mickrobot_description bringup_classic.launch.py '
            f'mapping_prior_file:={SESSION}/initial_pose.yaml'
        ),
    ),
    (
        '2-Localize+Nav2',
        (
            f'ros2 launch navigation_pkg navigation_humble.launch.py '
            f'start_patrol:=false rviz:=false '
            f'session_dir:={SESSION}'
        ),
    ),
    (
        '3-Patrol+RViz',
        (
            f'ros2 launch navigation_pkg patrol_rviz.launch.py '
            f'session_dir:={SESSION}'
        ),
    ),
]


def _shell_command(launch_cmd: str) -> str:
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
    return subprocess.Popen(
        [terminal, '-e', 'bash', '-lc', shell_cmd],
        cwd=str(ROOT),
    )


def _open_tmux(steps: list[tuple[str, str]], delay: float) -> None:
    if not shutil.which('tmux'):
        print('[launch_navigation] 未找到图形终端，也未安装 tmux。', file=sys.stderr)
        sys.exit(1)
    session = 'robothomework_navigation'
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
        print(f'[launch_navigation] tmux 窗口 {idx + 1}/3: {title}')
    subprocess.check_call(['tmux', 'select-window', '-t', f'{session}:0'])
    print(f'[launch_navigation] 附加会话: tmux attach -t {session}')
    subprocess.check_call(['tmux', 'attach', '-t', session])


def main() -> int:
    parser = argparse.ArgumentParser(description='分 3 个终端依次启动航点巡逻导航')
    parser.add_argument('--delay', type=float, default=6.0, help='相邻终端启动间隔（秒）')
    parser.add_argument('--kill-first', action='store_true', help='启动前清理仿真进程')
    parser.add_argument('--tmux', action='store_true', help='使用 tmux 代替图形终端')
    args = parser.parse_args()

    if args.kill_first and KILL_SCRIPT.is_file():
        print('[launch_navigation] 清理仿真进程...')
        subprocess.run(['bash', str(KILL_SCRIPT)], cwd=str(ROOT), check=False)

    if args.tmux:
        _open_tmux(STEPS, args.delay)
        return 0

    terminal = _detect_terminal()
    if terminal is None:
        print('[launch_navigation] 未找到图形终端，尝试 tmux...')
        _open_tmux(STEPS, args.delay)
        return 0

    procs: list[subprocess.Popen] = []
    for idx, (title, cmd) in enumerate(STEPS):
        if idx > 0:
            time.sleep(args.delay)
        print(f'[launch_navigation] 启动 {idx + 1}/3: {title}')
        procs.append(_open_terminal(terminal, title, cmd))

    print('[launch_navigation] 三个终端已启动。关闭任一窗口不影响其余窗口。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
