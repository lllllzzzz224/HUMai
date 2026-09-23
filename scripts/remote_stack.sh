#!/usr/bin/env bash
# ==============================================================================
# RM65 机械臂 & RealSense 视觉抓取栈一键管理脚本 (Ubuntu 端)
# 运行环境: Ubuntu 24.04, ROS 2 Jazzy, RealMan RM65-B, RealSense D435
# ==============================================================================

set -eo pipefail

SESSION_NAME="rm_stack"
UBUNTU_ROOT="/home/li/hand_eye_calibration"
SCRIPTS_DIR="${UBUNTU_ROOT}/scripts"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="/home/li/ros2_ws/install/setup.bash"
YOLO_PYTHON="/home/li/anaconda3/envs/yolo11/bin/python"
ROBOT_IP="192.168.1.18"
ROBOT_NET_IF_DEFAULT="enx00e04c3a4178"
ROBOT_HOST_IP="192.168.1.100/24"
export ROS_DOMAIN_ID=42

detect_robot_net_if() {
    if [ -n "${ROBOT_NET_IF:-}" ] && ip link show "${ROBOT_NET_IF}" >/dev/null 2>&1; then
        echo "${ROBOT_NET_IF}"
        return
    fi
    local cand
    cand=$(ip -br link | awk '{print $1}' | grep -E '^enx|^eth[1-9]|^enp[2-9]|^usb' | head -n 1)
    if [ -n "$cand" ]; then
        echo "$cand"
    else
        echo "${ROBOT_NET_IF_DEFAULT}"
    fi
}

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $*"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
log_err()  { echo -e "${RED}[ERROR]${NC} $*"; }
log_step() { echo -e "${CYAN}[STEP]${NC} $*"; }

ensure_ros_env() {
    if [ -f "$ROS_SETUP" ]; then
        source "$ROS_SETUP"
    else
        log_err "未找到 ROS 2 环境: $ROS_SETUP"
        exit 1
    fi
    if [ -f "$WS_SETUP" ]; then
        source "$WS_SETUP"
    else
        log_err "未找到工作空间环境: $WS_SETUP"
        exit 1
    fi
    export ROS_DOMAIN_ID=42
}

# ==========================================
# 1. 硬件与网络自检 (doctor)
# ==========================================
do_doctor() {
    echo -e "${BLUE}=========================================${NC}"
    echo -e "${BLUE}       RM65 机械臂系统端到端自检         ${NC}"
    echo -e "${BLUE}=========================================${NC}"

    local all_passed=true

    # 1.1 检查机械臂网络连接
    local net_if
    net_if=$(detect_robot_net_if)
    log_step "1. 检查机械臂网络连通性 (目标 IP: ${ROBOT_IP}, 探测网卡: ${net_if})..."
    if ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
        echo -e "   [PASS] 机械臂 IP 通信正常 (${ROBOT_IP})"
    else
        log_warn "   [FAIL] 无法 Ping 通机械臂 IP (${ROBOT_IP})"
        log_step "   正在尝试配置网卡 ${net_if} IP 为 ${ROBOT_HOST_IP}..."
        if sudo ip address replace "${ROBOT_HOST_IP}" dev "${net_if}" 2>/dev/null && \
           sudo ip link set "${net_if}" up 2>/dev/null; then
            sleep 1
            if ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
                echo -e "   [FIXED] 网卡已自动恢复，机械臂通信成功！"
            else
                log_err "   [FAIL] 自动配置后仍无法 Ping 通，请检查网线插头或机械臂电源。"
                all_passed=false
            fi
        else
            log_err "   [FAIL] 网卡 ${net_if} 不存在或权限不足。"
            all_passed=false
        fi
    fi

    # 1.2 检查 RealSense 相机 USB
    log_step "2. 检查 RealSense 深度相机连接..."
    if lsusb | grep -qi "RealSense"; then
        local cam_dev
        cam_dev=$(lsusb | grep -i "RealSense" | head -n 1)
        echo -e "   [PASS] 检测到相机设备: ${cam_dev}"
    else
        log_err "   [FAIL] 未在 USB 总线上发现 RealSense 相机！"
        all_passed=false
    fi

    # 1.3 检查 Python/Conda 虚拟环境
    log_step "3. 检查 YOLO 虚拟环境..."
    if [ -x "${YOLO_PYTHON}" ]; then
        echo -e "   [PASS] YOLO Python 存在: ${YOLO_PYTHON}"
    else
        log_err "   [FAIL] 未找到 YOLO Python 解释器: ${YOLO_PYTHON}"
        all_passed=false
    fi

    # 1.4 检查旧残留进程
    log_step "4. 检查后台残留孤儿进程..."
    local stale_procs
    stale_procs=$(ps aux | grep -E "rm_driver|rm_control|realsense2_camera|apple_center_localizer|static_transform_publisher" | grep -v grep || true)
    if [ -n "$stale_procs" ]; then
        local count
        count=$(echo "$stale_procs" | wc -l)
        log_warn "   [WARN] 发现 $count 个可能残留的 ROS 进程正在运行。"
        echo "$stale_procs" | awk '{print "      - PID: "$2" Cmd: "$11}' | head -n 5
        log_warn "   建议启动前执行一次 stop 清理。"
    else
        echo -e "   [PASS] 无冲突残留进程，环境干净。"
    fi

    echo -e "${BLUE}=========================================${NC}"
    if [ "$all_passed" = true ]; then
        log_info "所有核心硬件与网络诊断项均已通过！"
        return 0
    else
        log_err "诊断发现异常，请核对上述报错项！"
        return 1
    fi
}

# ==========================================
# 2. 清理与停止 (stop / clean)
# ==========================================
do_stop() {
    log_step "正在停止 RM65 机械臂与视觉服务栈..."

    # 2.1 关闭 tmux 会话
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log_info "向 tmux 会话 [${SESSION_NAME}] 发送平稳终止信号 (SIGINT)..."
        tmux send-keys -t "$SESSION_NAME" C-c 2>/dev/null || true
        sleep 2
        tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
        log_info "tmux 会话已销毁。"
    else
        log_info "tmux 会话 [${SESSION_NAME}] 未在运行。"
    fi

    # 2.2 彻底清理残留孤儿进程
        log_info "正在清理可能残留的孤儿进程..."
        local pids
        pids=$(pgrep -f "apple_hand_eye_all.launch.py|rm_driver|rm_control|realsense2_camera_node|apple_center_localizer_v2.py|camera_monitor.py|static_transform_publisher|move_group|rviz2" || true)
        if [ -n "$pids" ]; then
            echo "$pids" | xargs -r kill -INT 2>/dev/null || true
            sleep 1
            pids_left=$(pgrep -f "apple_hand_eye_all.launch.py|rm_driver|rm_control|realsense2_camera_node|apple_center_localizer_v2.py|camera_monitor.py|static_transform_publisher|move_group|rviz2" || true)
            if [ -n "$pids_left" ]; then
                echo "$pids_left" | xargs -r kill -9 2>/dev/null || true
            fi
            log_info "残留进程清理完成。"
        else
            log_info "无孤儿进程残留。"
        fi

    # 2.3 重启 ros2 daemon 避免残余缓存
    ensure_ros_env
    ros2 daemon stop >/dev/null 2>&1 || true
    log_info "ROS 2 daemon 已重置。"
    log_info "系统已恢复干净初始状态。"
}

# ==========================================
# 3. 启动基础设施栈 (start)
# ==========================================
do_start() {
    local target="${1:-orange}"
    log_step "准备启动机械臂与视觉栈 (目标水果: ${target})..."

    # 3.1 先执行一次清理，确保状态原子化
    do_stop

    # 3.2 检查网络与硬件
    local net_if
    net_if=$(detect_robot_net_if)
    if ! ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
        log_warn "机械臂 IP 不通，尝试在网卡 ${net_if} 上配置..."
        sudo ip address replace "${ROBOT_HOST_IP}" dev "${net_if}" 2>/dev/null || true
        sudo ip link set "${net_if}" up 2>/dev/null || true
        sleep 1
        if ! ping -c 1 -W 1 "${ROBOT_IP}" >/dev/null 2>&1; then
            log_err "无法连接机械臂 (${ROBOT_IP})，启动终止！"
            exit 1
        fi
    fi

    # 3.3 创建 tmux 会话并启动 launch 文件
    log_step "在 tmux 会话 [${SESSION_NAME}] 中启动主 launch 文件..."
    tmux new-session -d -s "$SESSION_NAME" -n "stack"

    local launch_cmd="source ${ROS_SETUP} && source ${WS_SETUP} && export ROS_DOMAIN_ID=42 && cd ${UBUNTU_ROOT}/apple_pick_v2 && ros2 launch apple_hand_eye_all.launch.py target:=${target} allow_trajectory_execution:=true"
    tmux send-keys -t "${SESSION_NAME}:stack" "${launch_cmd}" C-m

    log_info "启动命令已发送至 tmux 窗口。正在等待核心节点就绪 (最长等待 30 秒)..."

    ensure_ros_env

    # 3.4 轮询等待核心话题就绪
    local ready=false
    local start_time
    start_time=$(date +%s)
    local timeout=30

    while [ $(($(date +%s) - start_time)) -lt $timeout ]; do
        local elapsed=$(($(date +%s) - start_time))
        # 检查关节状态与相机数据
        local cur_topics
        cur_topics=$(ros2 topic list 2>/dev/null || true)
        if echo "$cur_topics" | grep -q "^/joint_states$"; then
            if echo "$cur_topics" | grep -q "^/camera/camera/color/image_raw$"; then
                if echo "$cur_topics" | grep -q "^/apple_pick_v2/apple_center$"; then
                    ready=true
                    break
                fi
            fi
        fi
        echo -ne "\r   正在等待节点上线... [已耗时: ${elapsed}s / ${timeout}s]"
        sleep 1
    done
    echo ""

    if [ "$ready" = true ]; then
        log_info "所有核心节点（驱动、MoveIt、RealSense 相机、手眼 TF、YOLO 识别）已全部就绪！"
        echo -e "   - 机械臂控制器: ${GREEN}READY${NC}"
        echo -e "   - RealSense 相机: ${GREEN}READY${NC}"
        echo -e "   - 手眼坐标 TF:   ${GREEN}READY${NC}"
        echo -e "   - YOLO 目标识别: ${GREEN}READY (目标: ${target})${NC}"
        echo -e "   提示: 使用 '${0} status' 可查看当前识别与位姿数据。"
    else
        log_warn "部分话题在 ${timeout} 秒内未完全就绪，请检查 tmux 日志: '${0} logs'"
    fi
}

# ==========================================
# 4. 运行状态查询 (status)
# ==========================================
do_status() {
    set +e
    echo -e "${BLUE}=========================================${NC}"
    echo -e "${BLUE}       RM65 机械臂运行状态监控           ${NC}"
    echo -e "${BLUE}=========================================${NC}"

    ensure_ros_env

    # 检查 tmux 会话
    if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo -e "tmux 会话 [${SESSION_NAME}]: ${GREEN}RUNNING${NC}"
    else
        echo -e "tmux 会话 [${SESSION_NAME}]: ${RED}STOPPED${NC}"
    fi

    # 检查关键话题
    echo -e "\n关键话题心跳状态:"
    local topics=(
        "/joint_states|机械臂关节状态"
        "/camera/camera/color/image_raw|RealSense 彩色图像"
        "/camera/camera/aligned_depth_to_color/image_raw|RealSense 深度图像"
        "/apple_pick_v2/apple_center|目标中心坐标(base_link)"
    )

    local live_topics
    live_topics=$(ros2 topic list 2>/dev/null || true)

    local center_online=false
    for item in "${topics[@]}"; do
        local top_name="${item%%|*}"
        local top_desc="${item##*|}"
        if echo "$live_topics" | grep -q "^${top_name}$"; then
            echo -e "  [${GREEN}ONLINE${NC}] ${top_desc} (${top_name})"
            if [ "$top_name" = "/apple_pick_v2/apple_center" ]; then
                center_online=true
            fi
        else
            echo -e "  [${RED}OFFLINE${NC}] ${top_desc} (${top_name})"
        fi
    done

    # 读取当前最新目标中心 (仅在话题在线时读取)
    echo -e "\n当前视觉识别目标位姿:"
    if [ "$center_online" = true ]; then
        local center_info
        center_info=$(timeout 5 ros2 topic echo /apple_pick_v2/apple_center --once 2>/dev/null || true)
        if [ -n "$center_info" ]; then
            local px py pz
            px=$(echo "$center_info" | grep -A 3 "position:" | grep "x:" | awk '{print $2}')
            py=$(echo "$center_info" | grep -A 3 "position:" | grep "y:" | awk '{print $2}')
            pz=$(echo "$center_info" | grep -A 3 "position:" | grep "z:" | awk '{print $2}')
            echo -e "  目标位置: [${GREEN}X=${px}, Y=${py}, Z=${pz}${NC}] 米 (基坐标系 base_link)"
        else
            echo -e "  ${YELLOW}话题在线但暂未捕获到有效帧。${NC}"
        fi
    else
        echo -e "  ${YELLOW}视觉检测节点未启动或未识别到目标。${NC}"
    fi
    echo -e "${BLUE}=========================================${NC}"
    return 0
}

# ==========================================
# 5. 执行抓取任务 (grasp)
# ==========================================
do_grasp() {
    local mode="${1:-preview}"
    log_step "准备执行抓取任务 (模式: ${mode})..."

    ensure_ros_env

    # 安全检查：检查底层栈是否在线
    local live_topics
    live_topics=$(ros2 topic list 2>/dev/null || true)
    if ! echo "$live_topics" | grep -q "^/joint_states$"; then
        log_err "底层机械臂驱动未运行！请先执行 '${0} start' 拉起基础设施。"
        exit 1
    fi

    cd "${UBUNTU_ROOT}/apple_pick_v2"

    case "$mode" in
        preview)
            log_info "【安全模式】仅执行实时视觉识别与 MoveIt 轨迹预检 (不动作机械臂、不闭爪)..."
            ${YOLO_PYTHON} single_apple_full_grasp.py --preflight-only --operator-confirmed --place-target basin
            ;;
        verify)
            log_info "【低速到位验证】真机执行 SCAN -> PREGRASP -> VERIFY (到位即停，不抓取)..."
            ${YOLO_PYTHON} single_apple_full_grasp.py --verify-only --execute --operator-confirmed
            ;;
        single)
            log_warn "【真机单轮抓取入盆】即将执行完整抓取、上抬与转运入盆！"
            echo -e "${RED}安全确认：请确保机械臂工作区内无人员，示教器急停在手！${NC}"
            ${YOLO_PYTHON} single_apple_full_grasp.py --execute --operator-confirmed --fully-autonomous --place-target basin
            ;;
        continuous)
            log_warn "【真机连续抓取】进入连续循环抓取模式！"
            echo -e "${RED}安全确认：机械臂回到 SCAN 后方可换果实，示教器急停在手！${NC}"
            ${YOLO_PYTHON} single_apple_full_grasp.py --continuous --execute --operator-confirmed
            ;;
        *)
            log_err "未知抓取模式: ${mode}。可选: preview | verify | single | continuous"
            exit 1
            ;;
    esac
}

# ==========================================
# 6. 盆子对位示教 (align-basin)
# ==========================================
do_align_basin() {
    log_step "准备执行盆子对位示教..."
    ensure_ros_env
    local live_topics
    live_topics=$(ros2 topic list 2>/dev/null || true)
    if ! echo "$live_topics" | grep -q "^/joint_states$"; then
        log_err "底层机械臂驱动未运行！请先执行 '${0} start' 拉起基础设施。"
        exit 1
    fi
    cd "${UBUNTU_ROOT}/apple_pick_v2"
    ${YOLO_PYTHON} show_basin_position.py --execute
}

# ==========================================
# 7. 查看日志 (logs / attach)
# ==========================================
do_logs() {
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log_err "tmux 会话 [${SESSION_NAME}] 未在运行。"
        exit 1
    fi
    log_info "正在附着 (attach) 到 tmux 会话 [${SESSION_NAME}]..."
    log_info "提示: 退出 tmux 观察请按快捷键 'Ctrl+b' 然后按 'd' (Detach)，不要按 Ctrl+C！"
    sleep 1
    tmux attach-session -t "$SESSION_NAME"
}

# ==========================================
# 7. 实时视觉监控大屏 (view)
# ==========================================
do_view() {
    log_step "正在检查/启动实时视觉监控大屏 (GUI + Web 流)..."
    ensure_ros_env

    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        log_warn "基础服务栈未运行，请先执行: $0 start"
        exit 1
    fi

    # 检查 monitor 窗口是否存在
    if ! tmux list-windows -t "$SESSION_NAME" 2>/dev/null | grep -q "monitor"; then
        log_info "正在创建 monitor 独立显示通道并启动窗口..."
        tmux new-window -t "$SESSION_NAME" -n monitor
        local mon_cmd="export ROS_DOMAIN_ID=42 && source ${ROS_SETUP} && export DISPLAY=:0 && export WAYLAND_DISPLAY=wayland-0 && export XDG_RUNTIME_DIR=/run/user/1000 && export XAUTHORITY=\$(ls -1 /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -n 1) && python3 -u ${SCRIPT_DIR}/camera_monitor.py --gui --web --port 5000"
        tmux send-keys -t "${SESSION_NAME}:monitor" "${mon_cmd}" C-m
        sleep 2
    else
        log_info "monitor 监控大屏已在后台运行中。"
    fi

    log_info "视觉监控大屏已就绪！"
    echo -e "  - ${GREEN}Ubuntu 主机显示屏${NC} : 已弹出独立实时画面 (快捷键: 'f' 全屏, 'q' 退出)"
    echo -e "  - ${GREEN}Mac / 本地浏览器${NC} : http://10.77.0.2:5000 (直接在浏览器看实时画面)"
}

# ==========================================
# 主入口参数解析
# ==========================================
cmd="${1:-help}"
shift || true

case "$cmd" in
    doctor)
        do_doctor "$@"
        ;;
    start)
        do_start "$@"
        ;;
    stop|clean)
        do_stop "$@"
        ;;
    status)
        do_status "$@"
        ;;
    view|monitor)
        do_view "$@"
        ;;
    grasp)
        do_grasp "$@"
        ;;
    align-basin|align)
        do_align_basin "$@"
        ;;
    logs|attach)
        do_logs "$@"
        ;;
    help|--help|-h)
        echo "用法: $0 {doctor|start [target]|stop|status|view|grasp [mode]|logs}"
        echo "  doctor          : 系统环境与硬件端到端自检"
        echo "  start [orange]  : 一键拉起基础设施栈 (可指定目标: apple | orange)"
        echo "  stop            : 一键平稳关闭并彻底清理孤儿进程"
        echo "  status          : 查看当前各节点心跳与目标识别坐标"
        echo "  view            : 在主机显示器弹窗显示实时画面，并在端口 5000 开启 Web 实时流"
        echo "  grasp [preview] : 执行抓取任务 (模式: preview | verify | single | continuous)"
        echo "  logs            : 附着到 tmux 会话查看实时日志"
        ;;
    *)
        log_err "未知命令: $cmd"
        echo "输入 '$0 help' 查看帮助。"
        exit 1
        ;;
esac
