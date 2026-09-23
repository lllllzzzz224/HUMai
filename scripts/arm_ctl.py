#!/usr/bin/env python3
"""
RM65 机械臂与 RealSense 抓取系统 - Mac 统一控制台 (arm_ctl)
------------------------------------------------------------
一键整合多终端连接、环境加载、硬件自检、状态监控、安全抓取与孤儿进程清理。
"""

import argparse
import os
import subprocess
import sys
import time

# 默认远程配置
DEFAULT_HOST = "humai@10.77.0.2"
REMOTE_SCRIPT = "/home/li/hand_eye_calibration/scripts/robot_stack_manager.sh"

# ANSI 颜色定义
class Color:
    HEADER = "\033[95m"
    BLUE = "\033[94m"
    CYAN = "\033[96m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"
    RESET = "\033[0m"


def run_cmd(cmd: list[str], check: bool = True, capture_output: bool = False) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, check=check, text=True, capture_output=capture_output)


def run_ssh(host: str, remote_cmd: str, interactive: bool = False, capture_output: bool = False) -> subprocess.CompletedProcess:
    ssh_cmd = ["ssh"]
    if interactive:
        ssh_cmd.append("-t")
    ssh_cmd.extend([host, remote_cmd])
    return subprocess.run(ssh_cmd, text=True, capture_output=capture_output)


# ==============================================================================
# 命令实现
# ==============================================================================

def cmd_doctor(host: str) -> None:
    """系统全链路自检"""
    print(f"\n{Color.BOLD}{Color.CYAN}=== [1/2] 正在检查 Mac 与 Ubuntu 主机连接 ==={Color.RESET}")
    host_ip = host.split("@")[-1]
    res = subprocess.run(["ping", "-c", "1", "-W", "1000", host_ip], capture_output=True)
    if res.returncode == 0:
        print(f" {Color.GREEN}✔ Mac 与 Ubuntu 主机 ({host_ip}) 网络直连正常{Color.RESET}")
    else:
        print(f" {Color.RED}✘ 无法 Ping 通 Ubuntu 主机 ({host_ip})，请检查网线连接！{Color.RESET}")
        return

    print(f"\n{Color.BOLD}{Color.CYAN}=== [2/2] 正在执行 Ubuntu 远端全链路自检 ==={Color.RESET}")
    run_ssh(host, f"{REMOTE_SCRIPT} doctor")


def cmd_start(host: str, target: str = "orange") -> None:
    """一键拉起基础设施栈"""
    print(f"\n{Color.BOLD}{Color.BLUE}🚀 正在启动 RM65 机械臂与视觉感知服务栈...{Color.RESET}")
    print(f"   目标类别: {Color.GREEN}{target}{Color.RESET}")
    print(f"   执行节点: 驱动(rm_driver) + 控制器(rm_control) + 相机(D435) + 手眼TF + MoveIt + YOLO")
    print(f"   底层管理: Ubuntu 后台独立 tmux 会话 [rm_stack]\n")
    
    run_ssh(host, f"{REMOTE_SCRIPT} start {target}")


def cmd_status(host: str) -> None:
    """查看运行状态与话题心跳"""
    print(f"\n{Color.BOLD}{Color.CYAN}📊 正在获取机械臂系统实时运行状态...{Color.RESET}")
    run_ssh(host, f"{REMOTE_SCRIPT} status")


def cmd_grasp(host: str, mode: str = "preview") -> None:
    """安全分级执行抓取任务"""
    modes_desc = {
        "preview": "仅视觉识别与轨迹预检 (不动作机械臂、不闭爪)",
        "verify": "低速到位验证 (SCAN -> PREGRASP -> VERIFY，到位即停)",
        "single": "真机单轮全流程抓取入盆 (识别 -> 抓取 -> 上抬 -> 转运入盆 -> 释放 -> 回位)",
        "continuous": "真机连续循环抓取入盆 (多果实接力采摘入盆)",
    }

    desc = modes_desc.get(mode, "未知模式")
    print(f"\n{Color.BOLD}🎯 准备执行抓取任务{Color.RESET}")
    print(f"   模式: {Color.CYAN}{mode}{Color.RESET} ({desc})")

    # 针对真机物理运动模式，强制弹出安全警告与二次确认
    if mode in ["single", "continuous", "verify"]:
        print(f"\n{Color.RED}{Color.BOLD}⚠️  【物理安全警告】{Color.RESET}")
        print(f"{Color.YELLOW}   - 机械臂即将进行物理轨迹运动！{Color.RESET}")
        print(f"{Color.YELLOW}   - 请确保机械臂运动范围内没有任何人员或障碍物！{Color.RESET}")
        print(f"{Color.YELLOW}   - 请确保急停按钮握在手中，随时做好急停准备！{Color.RESET}\n")

        confirm = input(f"请输入 {Color.BOLD}'yes'{Color.RESET} 确认执行，输入其他任意键取消: ").strip()
        if confirm.lower() != "yes":
            print(f"{Color.YELLOW}❌ 操作员已取消真机抓取。{Color.RESET}")
            return

    print(f"\n{Color.GREEN}▶ 正在启动抓取进程...{Color.RESET}\n")
    run_ssh(host, f"{REMOTE_SCRIPT} grasp {mode}", interactive=True)


def cmd_stop(host: str) -> None:
    """一键停止并彻底清理孤儿进程"""
    print(f"\n{Color.BOLD}{Color.YELLOW}🛑 正在安全停止系统并清理后台进程...{Color.RESET}")
    run_ssh(host, f"{REMOTE_SCRIPT} stop")


def cmd_logs(host: str) -> None:
    """附着到 tmux 会话查看实时日志"""
    print(f"\n{Color.BOLD}{Color.CYAN}📜 正在打开远端后台日志 (tmux attach)...{Color.RESET}")
    print(f"   {Color.YELLOW}提示: 退出日志观察请按快捷键 'Ctrl+b' 然后按 'd' (Detach)，切勿按 Ctrl+C！{Color.RESET}\n")
    time.sleep(1)
    run_ssh(host, f"{REMOTE_SCRIPT} logs", interactive=True)


def cmd_align_basin(host: str) -> None:
    """机械臂移动至落料点悬停，辅助操作员对位摆放盆子"""
    print(f"\n{Color.BOLD}{Color.CYAN}🥣 正在启动盆子对位示教向导 (Align Basin Guide)...{Color.RESET}")
    print(f"{Color.YELLOW}   - 机械臂将以安全慢速移动至落料点并张开夹爪{Color.RESET}")
    print(f"{Color.YELLOW}   - 到位后请将塑料盆推到两指中心正下方对齐{Color.RESET}")
    print(f"{Color.YELLOW}   - 对齐完成后按 Enter 让机械臂平缓返回空中 SCAN{Color.RESET}\n")

    confirm = input(f"请输入 {Color.BOLD}'yes'{Color.RESET} 确认执行对位，输入其他任意键取消: ").strip()
    if confirm.lower() != "yes":
        print(f"{Color.YELLOW}❌ 操作员已取消对位操作。{Color.RESET}")
        return

    run_ssh(host, f"{REMOTE_SCRIPT} align-basin", interactive=True)


def cmd_view(host: str) -> None:
    """启动/打开实时视觉监控大屏 (主机屏幕独立窗口 + Web 实时流)"""
    print(f"\n{Color.BOLD}{Color.CYAN}📺 正在启动/检查实时视觉监控大屏...{Color.RESET}")
    print(f"   {Color.GREEN}✔ Ubuntu 主机屏幕{Color.RESET} : 正在拉起独立图形窗口 (快捷键: 'f' 全屏, 'q' 退出)")
    print(f"   {Color.GREEN}✔ Mac 本地浏览器{Color.RESET} : 请在浏览器打开 {Color.BOLD}{Color.YELLOW}http://10.77.0.2:5000{Color.RESET} 即可实时查看！\n")
    run_ssh(host, f"{REMOTE_SCRIPT} view")


def cmd_ssh(host: str) -> None:
    """快速打开 SSH 交互终端"""
    print(f"\n{Color.CYAN}正在连接到 {host}...{Color.RESET}\n")
    run_ssh(host, "bash", interactive=True)


# ==============================================================================
# 交互式菜单 (TUI)
# ==============================================================================

def interactive_menu(host: str) -> None:
    while True:
        print("\n" + "=" * 54)
        print(f"{Color.BOLD}{Color.CYAN}       🤖 RM65 机械臂一键智能控制台 (Mac 终端)    {Color.RESET}")
        print("=" * 54)
        print(f" 目标主机: {Color.GREEN}{host}{Color.RESET}")
        print("-" * 54)
        print(f"  [{Color.BOLD}1{Color.RESET}] 🩺 系统诊断 (doctor)      - 检查网络、机械臂通信与相机")
        print(f"  [{Color.BOLD}2{Color.RESET}] 🚀 启动服务 (start)       - 一键拉起驱动、相机、MoveIt、YOLO")
        print(f"  [{Color.BOLD}3{Color.RESET}] 📊 状态监控 (status)      - 查看关节心跳与目标识别坐标")
        print(f"  [{Color.BOLD}4{Color.RESET}] 🎯 执行抓取 (grasp)       - 预检/单轮抓取/连续抓取 (带安全确认)")
        print(f"  [{Color.BOLD}5{Color.RESET}] 🥣 盆子对位 (align-basin) - 机械臂慢速移动至落料点，助你对齐盆子")
        print(f"  [{Color.BOLD}6{Color.RESET}] 🛑 安全停止 (stop)        - 一键平稳关闭并彻底清理孤儿进程")
        print(f"  [{Color.BOLD}7{Color.RESET}] 📜 实时日志 (logs)        - 附着到后台查看各节点实时输出")
        print(f"  [{Color.BOLD}8{Color.RESET}] 📺 视觉监控 (view)        - 在主机显示屏弹窗并在 Mac 浏览器开实时流")
        print(f"  [{Color.BOLD}9{Color.RESET}] 💻 SSH 终端 (ssh)         - 快速进入 Ubuntu 命令行")
        print(f"  [{Color.BOLD}0{Color.RESET}] 🚪 退出控制台")
        print("=" * 54)

        choice = input(f"请选择操作 [0-9]: ").strip()

        if choice == "1":
            cmd_doctor(host)
        elif choice == "2":
            target = input("请输入目标水果类型 (apple/orange，默认: orange): ").strip()
            target = target if target else "orange"
            cmd_start(host, target)
        elif choice == "3":
            cmd_status(host)
        elif choice == "4":
            print("\n可选抓取模式:")
            print("  1. preview    : 仅轨迹预检与位姿估算 (无真机动作，最安全)")
            print("  2. verify     : 低速到位验证 (仅移动到预抓取点检验误差)")
            print("  3. single     : 真机完整单轮抓取入盆 (抓取并放入盆中)")
            print("  4. continuous : 真机连续多轮抓取")
            m_choice = input("请选择模式 [1-4，默认 1]: ").strip()
            mode_map = {"1": "preview", "2": "verify", "3": "single", "4": "continuous"}
            mode = mode_map.get(m_choice, "preview")
            cmd_grasp(host, mode)
        elif choice == "5":
            cmd_align_basin(host)
        elif choice == "6":
            cmd_stop(host)
        elif choice == "7":
            cmd_logs(host)
        elif choice == "8":
            cmd_view(host)
        elif choice == "9":
            cmd_ssh(host)
        elif choice in ["0", "q", "exit"]:
            print(f"\n{Color.CYAN}再见！{Color.RESET}\n")
            sys.exit(0)
        else:
            print(f"{Color.RED}无效选择，请重新输入。{Color.RESET}")


# ==============================================================================
# 入口函数
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="RM65 机械臂与视觉感知抓取系统 Mac 统一控制台",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default=os.getenv("ROBOT_HOST", DEFAULT_HOST), help=f"远程主机地址 (默认: {DEFAULT_HOST})")

    subparsers = parser.add_subparsers(dest="command", help="子命令")

    subparsers.add_parser("doctor", help="全链路环境与硬件自检")

    p_start = subparsers.add_parser("start", help="一键拉起基础设施栈")
    p_start.add_argument("--target", default="orange", choices=["orange", "apple", "all"], help="目标水果类型 (默认: orange)")

    subparsers.add_parser("status", help="查询系统节点状态与目标识别坐标")

    subparsers.add_parser("view", help="在主机显示屏弹窗显示实时画面，并在端口 5000 开启 Web 实时流")

    p_grasp = subparsers.add_parser("grasp", help="安全分级触发抓取任务")
    p_grasp.add_argument("--mode", default="preview", choices=["preview", "verify", "single", "continuous"], help="抓取模式 (默认: preview)")

    subparsers.add_parser("align-basin", help="机械臂落料点对位示教 (协助操作员对齐盆子)")
    subparsers.add_parser("stop", help="一键平稳关闭系统并清理后台进程")
    subparsers.add_parser("logs", help="附着到 tmux 查看实时日志")
    subparsers.add_parser("ssh", help="快速进入 Ubuntu SSH 交互终端")

    args = parser.parse_args()

    if not args.command:
        # 无子命令时默认进入交互式菜单
        interactive_menu(args.host)
        return

    if args.command == "doctor":
        cmd_doctor(args.host)
    elif args.command == "start":
        cmd_start(args.host, args.target)
    elif args.command == "status":
        cmd_status(args.host)
    elif args.command == "view":
        cmd_view(args.host)
    elif args.command == "grasp":
        cmd_grasp(args.host, args.mode)
    elif args.command == "align-basin":
        cmd_align_basin(args.host)
    elif args.command == "stop":
        cmd_stop(args.host)
    elif args.command == "logs":
        cmd_logs(args.host)
    elif args.command == "ssh":
        cmd_ssh(args.host)


if __name__ == "__main__":
    main()
