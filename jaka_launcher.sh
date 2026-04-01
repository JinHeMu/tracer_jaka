#!/usr/bin/env bash
# ═══════════════════════════════════════════════════════════════
#  JAKA 机器人系统 TUI 启动器  (Pure Bash + ANSI)
#  依赖: bash ≥ 4.0, tput, kill, stty  (均为系统标配)
#
#  键盘操作:
#    ↑ / ↓      切换条目
#    ← / →      切换分类 Tab
#    Enter       启动 / 执行
#    K           终止选中进程
#    A           终止所有进程
#    L           查看选中进程日志
#    Q           退出
# ═══════════════════════════════════════════════════════════════

# ──────────────────────────────────────────────
#  用户配置区（按需修改）
# ──────────────────────────────────────────────
ROS2_WS="${HOME}/tracer_jaka"
ROS2_DISTRO="humble"
PCD_DIR="${ROS2_WS}/src/point_cloud_trajectory/pcd"
NODE_DIR="${ROS2_WS}/src/point_cloud_trajectory"

# Source 前缀（每条命令自动携带）
ROS_SETUP="/opt/ros/${ROS2_DISTRO}/setup.bash"
WS_SETUP="${ROS2_WS}/install/setup.bash"
SRC=""
[[ -f "$ROS_SETUP" ]] && SRC="source ${ROS_SETUP} && "
[[ -f "$WS_SETUP"  ]] && SRC="${SRC}source ${WS_SETUP} && "

# ──────────────────────────────────────────────
#  ANSI 颜色 / 样式
# ──────────────────────────────────────────────
ESC=$'\033'
RESET="${ESC}[0m"
BOLD="${ESC}[1m"
DIM="${ESC}[2m"

# 前景色
FG_BLACK="${ESC}[30m";  FG_RED="${ESC}[31m";     FG_GREEN="${ESC}[32m"
FG_YELLOW="${ESC}[33m"; FG_BLUE="${ESC}[34m";    FG_MAGENTA="${ESC}[35m"
FG_CYAN="${ESC}[36m";   FG_WHITE="${ESC}[37m";   FG_GRAY="${ESC}[90m"
FG_LRED="${ESC}[91m";   FG_LGREEN="${ESC}[92m";  FG_LYELLOW="${ESC}[93m"
FG_LBLUE="${ESC}[94m";  FG_LMAGENTA="${ESC}[95m";FG_LCYAN="${ESC}[96m"

# 背景色
BG_BLACK="${ESC}[40m";  BG_RED="${ESC}[41m";     BG_GREEN="${ESC}[42m"
BG_YELLOW="${ESC}[43m"; BG_BLUE="${ESC}[44m";    BG_MAGENTA="${ESC}[45m"
BG_CYAN="${ESC}[46m";   BG_WHITE="${ESC}[47m";   BG_GRAY="${ESC}[100m"
BG_LBLUE="${ESC}[104m"

# ──────────────────────────────────────────────
#  条目数据库
#  格式: "TAB_INDEX|NAME|DESC|CMD|TYPE|ONESHOT"
#    TYPE   : python | launch | node | service | tool
#    ONESHOT: 1=可重复启动  0=防止重复
# ──────────────────────────────────────────────
declare -a TABS=("📦 离线处理" "🚀 Launch" "🤖 节点" "📡 服务/话题" "🔧 系统工具")

declare -a ENTRIES=(
  # ── Tab 0: 离线处理 ──
  "0|box_crop.py|包围盒裁剪原始点云 → cropped_cloud.pcd|cd ${PCD_DIR} && python3 box_crop.py|python|0"
  "0|pre_process.py|降采样→滤波→法线估计→Poisson网格重建|cd ${PCD_DIR} && python3 pre_process.py|python|0"
  "0|mesh_coverage_path.py|Boustrophedon覆盖路径规划 → coverage_path.csv|cd ${PCD_DIR} && python3 mesh_coverage_path.py|python|0"

  # ── Tab 1: Launch ──
  "1|real.launch.py|完整真机启动: MoveIt+ros2_control+RViz2|${SRC}ros2 launch tracer_jaka_moveit_config real.launch.py|launch|0"
  "1|jaka_servo_example.launch.py|Servo示例: MoveIt Servo + Joy手柄遥控|${SRC}ros2 launch tracer_jaka_moveit_config jaka_servo_example.launch.py|launch|0"

  # ── Tab 2: 节点 ──
  "2|path_visualizer_node.py|发布路径/法向/点云到RViz2|${SRC}python3 ${NODE_DIR}/path_visualizer_node.py|node|0"
  "2|servo_path_tracker.py|CSV路径点→末端速度指令(P控制器)|${SRC}python3 ${NODE_DIR}/servo_path_tracker.py|node|0"
  "2|force_admittance_servo_node|力导纳控制器 125Hz (路径跟踪+恒力)|${SRC}ros2 run force_admittance_servo force_admittance_servo_node|node|0"

  # ── Tab 3: 服务/话题 ──
  "3|开始路径跟踪|调用 /path_servo/start 触发路径跟踪|${SRC}ros2 service call /path_servo/start std_srvs/srv/Trigger '{}'|service|1"
  "3|停止路径跟踪|调用 /path_servo/stop 停止路径跟踪|${SRC}ros2 service call /path_servo/stop std_srvs/srv/Trigger '{}'|service|1"
  "3|使能力控节点|发布 True → enable 使能导纳控制|${SRC}ros2 topic pub --once /force_admittance_servo/enable std_msgs/msg/Bool '{data: true}'|service|1"
  "3|禁用力控节点|发布 False → enable 禁用导纳控制|${SRC}ros2 topic pub --once /force_admittance_servo/enable std_msgs/msg/Bool '{data: false}'|service|1"
  "3|监听 FTS 力矩|ros2 topic echo /tcp_fts_sensor/wrench|${SRC}ros2 topic echo /tcp_fts_sensor/wrench|service|0"
  "3|监听关节状态|ros2 topic echo /joint_states|${SRC}ros2 topic echo /joint_states|service|0"

  # ── Tab 4: 系统工具 ──
  "4|colcon build (全量)|编译整个工作空间|cd ${ROS2_WS} && source ${ROS_SETUP} && colcon build --symlink-install|tool|0"
  "4|colcon build (force_admittance)|仅编译力控节点包|cd ${ROS2_WS} && source ${ROS_SETUP} && colcon build --symlink-install --packages-select force_admittance_servo|tool|0"
  "4|RViz2|单独启动 RViz2|${SRC}rviz2|tool|0"
  "4|ros2 node list|列出当前所有在线 ROS2 节点|${SRC}ros2 node list|tool|1"
  "4|ros2 topic list|列出当前所有话题|${SRC}ros2 topic list|tool|1"
)

# ──────────────────────────────────────────────
#  运行时状态
# ──────────────────────────────────────────────
cur_tab=0       # 当前 Tab 索引
cur_row=0       # 当前条目（在该 Tab 内的索引）
flash_msg=""    # 底部闪烁消息
flash_time=0

declare -a PROC_NAMES=()   # 进程名称列表
declare -a PROC_PIDS=()    # 进程 PID
declare -a PROC_STATUS=()  # running | done | error
declare -a PROC_LOGS=()    # 日志文件路径
declare -a PROC_START=()   # 启动时间戳

LOG_DIR="/tmp/jaka_launcher_logs"
mkdir -p "$LOG_DIR"

# ──────────────────────────────────────────────
#  辅助函数：按 Tab 过滤条目，返回索引数组
# ──────────────────────────────────────────────
get_tab_entries() {
  local tab=$1
  local -n _out=$2
  _out=()
  for i in "${!ENTRIES[@]}"; do
    IFS='|' read -r t _ <<< "${ENTRIES[$i]}"
    [[ "$t" == "$tab" ]] && _out+=("$i")
  done
}

parse_entry() {
  # 用法: parse_entry INDEX  → 设置 e_tab e_name e_desc e_cmd e_type e_oneshot
  IFS='|' read -r e_tab e_name e_desc e_cmd e_type e_oneshot <<< "${ENTRIES[$1]}"
}

# ──────────────────────────────────────────────
#  终端控制
# ──────────────────────────────────────────────
term_init() {
  tput civis          # 隐藏光标
  tput smcup          # 进入备用屏幕
  stty -echo          # 关闭输入回显
}

term_restore() {
  tput cnorm          # 显示光标
  tput rmcup          # 恢复主屏幕
  stty echo
}

move()  { tput cup "$1" "$2"; }
clear_screen() { tput clear; }
get_cols() { tput cols; }
get_rows() { tput lines; }

# 在指定行列打印（自动裁剪至宽度 $4）
print_at() {
  local row=$1 col=$2 text=$3 maxw=${4:-9999}
  move "$row" "$col"
  printf "%s" "${text:0:$maxw}"
}

# 带颜色打印
cprint() { printf "%s%s%s" "$1" "$2" "$RESET"; }

# 填充空格到指定宽度
pad() { printf "%-${2}s" "${1:0:$2}"; }

# 截断字符串
trunc() {
  local s="$1" n="$2"
  if (( ${#s} > n )); then
    echo "${s:0:$((n-1))}…"
  else
    echo "$s"
  fi
}

# ──────────────────────────────────────────────
#  进程管理
# ──────────────────────────────────────────────
launch_entry() {
  local idx=$1
  parse_entry "$idx"

  local logfile="${LOG_DIR}/${e_name//\//_}_$(date +%s).log"
  bash -c "$e_cmd" >"$logfile" 2>&1 &
  local pid=$!

  PROC_NAMES+=("$e_name")
  PROC_PIDS+=("$pid")
  PROC_STATUS+=("running")
  PROC_LOGS+=("$logfile")
  PROC_START+=("$(date +%s)")

  flash_msg="▶ 已启动: ${e_name}  [PID ${pid}]"
  flash_time=$(date +%s)
}

kill_proc() {
  local pidx=$1   # 进程数组下标
  local pid="${PROC_PIDS[$pidx]}"
  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    # 尝试杀死整个进程组
    kill -- -"$pid" 2>/dev/null || kill "$pid" 2>/dev/null
    PROC_STATUS[$pidx]="done"
    flash_msg="✗ 已终止: ${PROC_NAMES[$pidx]}"
    flash_time=$(date +%s)
  fi
}

kill_all_procs() {
  for i in "${!PROC_PIDS[@]}"; do
    [[ "${PROC_STATUS[$i]}" == "running" ]] && kill_proc "$i"
  done
  flash_msg="✗ 已终止所有进程"
  flash_time=$(date +%s)
}

# 更新所有进程状态（检查是否还在跑）
refresh_proc_status() {
  for i in "${!PROC_PIDS[@]}"; do
    if [[ "${PROC_STATUS[$i]}" == "running" ]]; then
      if ! kill -0 "${PROC_PIDS[$i]}" 2>/dev/null; then
        PROC_STATUS[$i]="done"
      fi
    fi
  done
}

# 找当前 Tab + 行对应的进程（最新的那个）
find_proc_for_entry() {
  local name="$1"
  local found=-1
  for i in "${!PROC_NAMES[@]}"; do
    [[ "${PROC_NAMES[$i]}" == "$name" ]] && found=$i
  done
  echo "$found"
}

elapsed_str() {
  local start=$1
  local now; now=$(date +%s)
  local secs=$(( now - start ))
  printf "%02d:%02d" $(( secs/60 )) $(( secs%60 ))
}

# ──────────────────────────────────────────────
#  类型颜色/图标
# ──────────────────────────────────────────────
type_color() {
  case "$1" in
    python)  echo "${FG_LGREEN}"  ;;
    launch)  echo "${FG_LYELLOW}" ;;
    node)    echo "${FG_LCYAN}"   ;;
    service) echo "${FG_LMAGENTA}";;
    tool)    echo "${FG_WHITE}"   ;;
    *)       echo "${FG_GRAY}"    ;;
  esac
}

type_badge() {
  case "$1" in
    python)  echo " PY " ;;
    launch)  echo "LNCH" ;;
    node)    echo "NODE" ;;
    service) echo " SVC" ;;
    tool)    echo "TOOL" ;;
    *)       echo "    " ;;
  esac
}

# ──────────────────────────────────────────────
#  绘制函数
# ──────────────────────────────────────────────
draw_all() {
  refresh_proc_status
  local COLS; COLS=$(get_cols)
  local ROWS; ROWS=$(get_rows)

  clear_screen

  draw_header "$COLS" "$ROWS"
  draw_tabs   "$COLS"
  draw_list   "$COLS" "$ROWS"
  draw_proc_bar "$COLS" "$ROWS"
  draw_statusbar "$COLS" "$ROWS"
}

draw_header() {
  local W=$1
  local title=" 🤖  JAKA 机器人系统启动器"
  local keys=" [↑↓]选择  [←→]切Tab  [Enter]启动  [K]终止  [A]全止  [L]日志  [Q]退出"
  move 0 0
  printf "${BG_BLUE}${FG_LCYAN}${BOLD}%-${W}s${RESET}" "$title"
  move 0 $(( W - ${#keys} - 1 ))
  printf "${BG_BLUE}${FG_GRAY}%s${RESET}" "${keys:0:$(( W - ${#title} - 1 ))}"
}

draw_tabs() {
  local W=$1
  move 1 0
  printf "${DIM}%${W}s${RESET}" "" | tr ' ' '─'
  move 2 0
  local x=1
  for i in "${!TABS[@]}"; do
    local label=" ${TABS[$i]} "
    if (( i == cur_tab )); then
      printf "${BG_CYAN}${FG_BLACK}${BOLD}%s${RESET}" "$label"
    else
      printf "${FG_GRAY}%s${RESET}" "$label"
    fi
    printf "${FG_GRAY}│${RESET}"
  done
  echo ""
  move 3 0
  printf "${DIM}%${W}s${RESET}" "" | tr ' ' '─'
}

draw_list() {
  local W=$1 ROWS=$2
  local list_top=4
  local list_bottom=$(( ROWS - 5 ))  # 留给进程栏和状态栏
  local visible=$(( (list_bottom - list_top) / 3 ))

  declare -a tab_idxs
  get_tab_entries "$cur_tab" tab_idxs

  # 滚动偏移
  local offset=0
  if (( cur_row >= visible )); then
    offset=$(( cur_row - visible + 1 ))
  fi

  local row=$list_top
  for (( vi=0; vi < visible && vi+offset < ${#tab_idxs[@]}; vi++ )); do
    local real_i=$(( vi + offset ))
    local eidx="${tab_idxs[$real_i]}"
    parse_entry "$eidx"

    local is_sel=0
    (( real_i == cur_row )) && is_sel=1

    # 查找进程状态
    local pidx; pidx=$(find_proc_for_entry "$e_name")
    local status_icon="○"
    local status_col="${FG_GRAY}"
    local elapsed_s=""
    if (( pidx >= 0 )); then
      case "${PROC_STATUS[$pidx]}" in
        running) status_icon="▶"; status_col="${FG_LGREEN}"; elapsed_s=$(elapsed_str "${PROC_START[$pidx]}") ;;
        done)    status_icon="✓"; status_col="${FG_GRAY}" ;;
        error)   status_icon="✗"; status_col="${FG_LRED}" ;;
      esac
    fi

    local badge; badge=$(type_badge "$e_type")
    local tcol;  tcol=$(type_color "$e_type")
    local name_trunc; name_trunc=$(trunc "$e_name" $(( W - 22 )))
    local desc_trunc; desc_trunc=$(trunc "$e_desc" $(( W - 6 )))

    if (( is_sel )); then
      # 高亮行 1
      move $row 0
      printf "${BG_LBLUE}${FG_WHITE}${BOLD} %s [%s] %-$((W-14))s ${RESET}" \
             "$status_icon" "$badge" "$name_trunc"
      [[ -n "$elapsed_s" ]] && {
        move $row $(( W - ${#elapsed_s} - 2 ))
        printf "${BG_LBLUE}${FG_LGREEN}%s${RESET}" "$elapsed_s"
      }
      # 高亮行 2（描述）
      move $(( row+1 )) 0
      printf "${BG_LBLUE}${FG_GRAY}     %-$((W-2))s${RESET}" "$desc_trunc"
    else
      # 普通行 1
      move $row 0
      printf " ${status_col}%s${RESET} ${tcol}[%s]${RESET} ${FG_WHITE}%-$((W-14))s${RESET}" \
             "$status_icon" "$badge" "$name_trunc"
      [[ -n "$elapsed_s" ]] && {
        move $row $(( W - ${#elapsed_s} - 2 ))
        printf "${FG_LGREEN}%s${RESET}" "$elapsed_s"
      }
      # 普通行 2（描述）
      move $(( row+1 )) 0
      printf "     ${FG_GRAY}%-$((W-6))s${RESET}" "$desc_trunc"
    fi

    # 分隔线（非最后一项）
    if (( vi+offset < ${#tab_idxs[@]}-1 )); then
      move $(( row+2 )) 0
      printf "${FG_GRAY}%${W}s${RESET}" "" | tr ' ' '·'
    fi

    row=$(( row + 3 ))
  done

  # 滚动提示
  if (( ${#tab_idxs[@]} > visible )); then
    local total=${#tab_idxs[@]}
    move $(( list_bottom - 1 )) $(( W - 18 ))
    printf "${FG_GRAY}[ %d / %d 条 ↑↓滚动 ]${RESET}" $(( cur_row+1 )) "$total"
  fi
}

draw_proc_bar() {
  local W=$1 ROWS=$2
  local bar_row=$(( ROWS - 4 ))
  move "$bar_row" 0
  printf "${DIM}%${W}s${RESET}" "" | tr ' ' '─'

  move $(( bar_row+1 )) 0
  printf "${FG_YELLOW}${BOLD} 进程: ${RESET}"

  if (( ${#PROC_NAMES[@]} == 0 )); then
    printf "${FG_GRAY}(暂无)${RESET}"
    return
  fi

  local x=9
  for i in "${!PROC_NAMES[@]}"; do
    local pname; pname=$(trunc "${PROC_NAMES[$i]}" 16)
    local badge elapsed=""
    case "${PROC_STATUS[$i]}" in
      running) badge="${FG_LGREEN}▶${RESET}"; elapsed=" $(elapsed_str "${PROC_START[$i]}")" ;;
      done)    badge="${FG_GRAY}✓${RESET}"   ;;
      error)   badge="${FG_LRED}✗${RESET}"   ;;
    esac
    local chunk="${badge} ${pname}${elapsed}  "
    # 粗略长度（不含 escape）
    local chunk_vis="${pname}${elapsed}   "
    if (( x + ${#chunk_vis} + 5 > W )); then
      printf "${FG_GRAY}+%d more${RESET}" $(( ${#PROC_NAMES[@]} - i ))
      break
    fi
    printf "%s" "$chunk"
    x=$(( x + ${#chunk_vis} + 5 ))
  done
}

draw_statusbar() {
  local W=$1 ROWS=$2
  local bar_row=$(( ROWS - 2 ))

  # 获取当前条目 cmd 预览
  declare -a tab_idxs
  get_tab_entries "$cur_tab" tab_idxs
  local cmd_preview=""
  if (( cur_row < ${#tab_idxs[@]} )); then
    parse_entry "${tab_idxs[$cur_row]}"
    cmd_preview=$(trunc "$e_cmd" $(( W - 8 )))
  fi

  move "$bar_row" 0
  printf "${DIM}%${W}s${RESET}" "" | tr ' ' '─'

  move $(( bar_row+1 )) 0

  # 闪烁消息（3秒内显示）
  local now; now=$(date +%s)
  if [[ -n "$flash_msg" ]] && (( now - flash_time < 3 )); then
    printf "${FG_LGREEN}${BOLD} ✦ %-$((W-4))s${RESET}" "$flash_msg"
  else
    flash_msg=""
    printf "${FG_GRAY} CMD: %-$((W-7))s${RESET}" "$cmd_preview"
  fi
}

# ──────────────────────────────────────────────
#  日志查看器（全屏）
# ──────────────────────────────────────────────
show_log_viewer() {
  local name="$1"
  local pidx; pidx=$(find_proc_for_entry "$name")

  if (( pidx < 0 )) || [[ -z "${PROC_LOGS[$pidx]}" ]]; then
    flash_msg="⚠ '${name}' 暂无日志"
    flash_time=$(date +%s)
    return
  fi

  local logfile="${PROC_LOGS[$pidx]}"
  local COLS; COLS=$(get_cols)
  local ROWS; ROWS=$(get_rows)

  # 用 less 打开日志（支持滚动）
  tput cnorm
  tput rmcup
  stty echo
  less +G --RAW-CONTROL-CHARS --no-lessopen \
       --prompt="  JAKA日志: ${name} │ q=退出" \
       "$logfile" 2>/dev/null || cat "$logfile"
  stty -echo
  tput smcup
  tput civis
}

# ──────────────────────────────────────────────
#  键盘读取（处理方向键转义序列）
# ──────────────────────────────────────────────
read_key() {
  local key
  IFS= read -r -s -n1 key
  if [[ "$key" == $'\x1b' ]]; then
    local seq1 seq2
    IFS= read -r -s -n1 -t 0.1 seq1 || true
    IFS= read -r -s -n1 -t 0.1 seq2 || true
    key="${key}${seq1}${seq2}"
  fi
  echo "$key"
}

# ──────────────────────────────────────────────
#  主循环
# ──────────────────────────────────────────────
main() {
  term_init
  trap 'kill_all_procs; term_restore; exit 0' EXIT INT TERM

  while true; do
    draw_all

    local key; key=$(read_key)

    declare -a tab_idxs
    get_tab_entries "$cur_tab" tab_idxs
    local tab_len=${#tab_idxs[@]}

    case "$key" in

      # ── 方向键 ──────────────────────────────
      $'\x1b[A')   # ↑
        (( cur_row > 0 )) && (( cur_row-- ))
        ;;
      $'\x1b[B')   # ↓
        (( cur_row < tab_len - 1 )) && (( cur_row++ ))
        ;;
      $'\x1b[C')   # →
        (( cur_tab < ${#TABS[@]} - 1 )) && (( cur_tab++ ))
        cur_row=0
        ;;
      $'\x1b[D')   # ←
        (( cur_tab > 0 )) && (( cur_tab-- ))
        cur_row=0
        ;;

      # ── Enter: 启动 ─────────────────────────
      $'\n'|$'\r'|"")
        if (( tab_len > 0 )); then
          local eidx="${tab_idxs[$cur_row]}"
          parse_entry "$eidx"
          local pidx; pidx=$(find_proc_for_entry "$e_name")
          if (( e_oneshot == 0 )) && (( pidx >= 0 )) && \
             [[ "${PROC_STATUS[$pidx]}" == "running" ]]; then
            flash_msg="⚠ '${e_name}' 已在运行，按 K 终止后再启动"
            flash_time=$(date +%s)
          else
            launch_entry "$eidx"
          fi
        fi
        ;;

      # ── K: 终止选中 ─────────────────────────
      k|K)
        if (( tab_len > 0 )); then
          parse_entry "${tab_idxs[$cur_row]}"
          local pidx; pidx=$(find_proc_for_entry "$e_name")
          if (( pidx >= 0 )) && [[ "${PROC_STATUS[$pidx]}" == "running" ]]; then
            kill_proc "$pidx"
          else
            flash_msg="⚠ '${e_name}' 未在运行"
            flash_time=$(date +%s)
          fi
        fi
        ;;

      # ── A: 终止所有 ─────────────────────────
      a|A)
        if (( ${#PROC_NAMES[@]} > 0 )); then
          kill_all_procs
        fi
        ;;

      # ── L: 查看日志 ─────────────────────────
      l|L)
        if (( tab_len > 0 )); then
          parse_entry "${tab_idxs[$cur_row]}"
          show_log_viewer "$e_name"
        fi
        ;;

      # ── Q: 退出 ─────────────────────────────
      q|Q)
        term_restore
        echo ""
        if (( ${#PROC_NAMES[@]} > 0 )); then
          echo -e "${FG_YELLOW}正在终止所有子进程...${RESET}"
          kill_all_procs
          sleep 0.3
        fi
        echo -e "${FG_LGREEN}Bye! 🤖${RESET}"
        exit 0
        ;;

      # ── Tab: 切换分类 ───────────────────────
      $'\t')
        (( cur_tab = (cur_tab + 1) % ${#TABS[@]} ))
        cur_row=0
        ;;

    esac
  done
}

# ──────────────────────────────────────────────
#  检查终端尺寸
# ──────────────────────────────────────────────
COLS=$(get_cols); ROWS=$(get_rows)
if (( COLS < 80 || ROWS < 20 )); then
  echo "⚠  终端太小（当前 ${COLS}×${ROWS}），建议至少 80×20，请放大后重试。"
  exit 1
fi

main
