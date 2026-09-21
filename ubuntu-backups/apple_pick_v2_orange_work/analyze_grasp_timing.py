#!/usr/bin/env python3
"""Read-only timing analysis of legacy continuous grasp JSON logs (no ROS)."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
from statistics import mean


def _time(value):
    try:
        return datetime.fromisoformat(value)
    except (ValueError, TypeError):
        return None


def _elapsed(start, end):
    if start is None or end is None:
        return None
    try:
        seconds = (end - start).total_seconds()
    except TypeError:
        return None
    return seconds if seconds >= 0 else None


def _event_time(record, step):
    for event in record.get('events', []):
        if event.get('step') == step and event.get('result') in (None, 'PASS'):
            return _time(event.get('time'))
    return None


def analyze_round(record):
    """Intervals are coarse observations, not isolated solver benchmarks."""
    start = _time(record.get('timestamp'))
    end = _time(record.get('finished_at'))
    preflight = _event_time(record, '全部路径预检')
    motion = _event_time(record, '自动开始 SCAN -> PREGRASP -> VERIFY')
    returned = _event_time(record, '返回固定鸟瞰位')
    return {
        'status': record.get('status', 'UNKNOWN'),
        'timestamp': record.get('timestamp'),
        'finished_at': record.get('finished_at'),
        'duration_s': _elapsed(start, end),
        'prepare_to_preflight_s': _elapsed(start, preflight),
        'post_preflight_to_motion_s': _elapsed(preflight, motion),
        'motion_to_return_s': _elapsed(motion, returned),
        'motion_to_finish_s': _elapsed(motion, end),
        'post_return_s': _elapsed(returned, end),
        'error': record.get('error'),
    }


def analyze_session(path):
    path = Path(path)
    record = json.loads(path.read_text(encoding='utf-8'))
    previous = _time(record.get('timestamp'))
    rows = []
    missing = []
    for entry in record.get('rounds', []):
        # Resolve relocated sessions within their results directory only.
        name = Path(entry.get('log', '')).name
        log = path.parent / name
        if not name or not log.is_file():
            missing.append(str(log))
            previous = None
            continue
        data = json.loads(log.read_text(encoding='utf-8'))
        row = analyze_round(data)
        row.update({
            'log': str(log.resolve()),
            'round_index': entry.get('round_index', data.get('round_index')),
            'attempt': entry.get('attempt', data.get('pre_motion_attempt', 1)),
            'wait_before_s': _elapsed(previous, _time(data.get('timestamp'))),
        })
        rows.append(row)
        previous = _time(data.get('finished_at'))
    return {
        'session': str(path.resolve()),
        'status': record.get('status', 'UNKNOWN'),
        'completed_rounds': record.get('completed_rounds'),
        'settings': record.get('settings', {}),
        'session_duration_s': _elapsed(
            _time(record.get('timestamp')), _time(record.get('finished_at'))),
        'terminal_idle_s': _elapsed(previous, _time(record.get('finished_at'))),
        'missing_logs': missing,
        'rounds': rows,
    }


def _number(value):
    return '未知' if value is None else f'{value:g}'


def analyze_directory(root):
    root = Path(root)
    sessions = [analyze_session(path) for path in sorted(root.glob('continuous_apple_grasp_*.json'))]
    linked = {Path(row['log']).name for session in sessions for row in session['rounds']}
    unlinked = []
    for path in sorted(root.glob('*round*.json')):
        if path.name in linked:
            continue
        record = json.loads(path.read_text(encoding='utf-8'))
        row = analyze_round(record)
        row.update({'log': str(path.resolve()), 'round_index': record.get('round_index'),
                    'attempt': record.get('pre_motion_attempt', 1), 'wait_before_s': None})
        unlinked.append(row)
    return {'sessions': sessions, 'unlinked_rounds': unlinked}


def markdown_report(sessions, unlinked_rounds=()):
    lines = [
        '# 历史连续抓取耗时分析', '',
        '来源为原始连续会话及逐轮 JSON；仅读取，没有修改日志或运行机械臂。', '',
        '**计时边界：**旧日志精度为整秒。准备段包含候选筛选、规划及相关调用，不能当作精确求解器时间；'
        '运动段包含局部重新规划、到位确认、夹爪等待与运动。轮间等待包含人工换位、目标稳定门禁和采样等待，无法仅凭日志分解。', '',
    ]
    successful = []
    for session in sessions:
        idle_label = ('最后一轮结束至会话退出' if session['rounds']
                      else '无逐轮记录的会话等待')
        lines.extend([
            f"## {Path(session['session']).name}", '',
            f"会话状态：{session['status']}；记录完成 {session['completed_rounds']} 轮；"
            f"会话总长 {_number(session['session_duration_s'])} 秒；"
            f"{idle_label} {_number(session['terminal_idle_s'])} 秒。", '',
            '| 轮次/尝试 | 轮前等待(s) | 准备至预检(s) | 预检后至运动(s) | 运动至回SCAN(s) | 轮总长(s) | 状态 |',
            '|---|---:|---:|---:|---:|---:|---|',
        ])
        for row in session['rounds']:
            values = [row[k] for k in ('wait_before_s', 'prepare_to_preflight_s',
                       'post_preflight_to_motion_s', 'motion_to_return_s', 'duration_s')]
            lines.append(f"| {row['round_index']}/{row['attempt']} | "
                         + ' | '.join(map(_number, values)) + f" | {row['status']} |")
            if row['status'] == 'PASS_PICK_RETURN_COMPLETE':
                successful.append(row)
        lines.append('')
        for row in session['rounds']:
            lines.append(f"- 第 {row['round_index']} 轮证据：[{Path(row['log']).name}]({row['log']})")
            if row['error']:
                lines.append(f"  错误：{row['error']}")
                if row['motion_to_finish_s'] is not None:
                    lines.append(f"  已观察到开始运动至故障记录结束 {_number(row['motion_to_finish_s'])} 秒；没有回到 SCAN 的完成事件。")
        for missing in session['missing_logs']:
            lines.append(f'- 缺失逐轮日志：{missing}，相邻间隔不作归因。')
        lines.append('')
    linked_pass_count = len(successful)
    if unlinked_rounds:
        lines.extend(['## 未关联到完整会话的逐轮记录', '',
                      '下列文件独立存在，未重复计入上面的会话；没有会话关联时不推断轮间等待。', '',
                      '| 文件 | 准备至预检(s) | 预检后至运动(s) | 轮总长(s) | 状态 / 原因 |',
                      '|---|---:|---:|---:|---|'])
        for row in unlinked_rounds:
            lines.append(f"| [{Path(row['log']).name}]({row['log']}) | "
                         f"{_number(row['prepare_to_preflight_s'])} | "
                         f"{_number(row['post_preflight_to_motion_s'])} | "
                         f"{_number(row['duration_s'])} | {row['status']} / {row['error'] or '无报错'} |")
            if row['status'] == 'PASS_PICK_RETURN_COMPLETE':
                successful.append(row)
        lines.append('')
    lines.extend(['## 已完成动作轮次统计', '',
                  f'共 {len(sessions)} 个会话，关联 {linked_pass_count} 份 PASS_PICK_RETURN_COMPLETE 逐轮记录；'
                  f'另有 {len(successful)-linked_pass_count} 份未关联完整会话的 PASS 逐轮记录。'
                  f'下表合计 {len(successful)} 份 PASS 记录，该状态不证明物理夹持成功。', '',
                  '| 项目 | 样本数 | 平均秒数 | 最小–最大秒数 |',
                  '|---|---:|---:|---:|'])
    for key, label in [('duration_s', '逐轮总长'),
                       ('prepare_to_preflight_s', '准备至预检'),
                       ('post_preflight_to_motion_s', '预检后至运动'),
                       ('motion_to_return_s', '运动至回SCAN')]:
        values = [r[key] for r in successful if r[key] is not None]
        if values:
            lines.append(f'| {label} | {len(values)} | {mean(values):.2f} | {min(values):g}–{max(values):g} |')
    lines.extend(['', '## 源码对应与优化优先级', '',
                  '1. 原 collect_apple() 每次清空队列并重新收集 10 个样本；视觉默认 2 Hz。连续流程 prepare_round() 后的 validate_frozen_target() 再调用 collect_target()，约 5 秒的复采等待与多轮日志一致。应以新鲜、稳定的持续缓存复核目标，不能直接删掉位置复核。',
                  '2. 原连续模式需要位置变化至少 30 mm、稳定 2 秒再倒计时 3 秒。轮前间隔并非纯规划开销，也不能全部认定为软件浪费。',
                  '3. 运动段占主要活动时间，但包含到位停留、局部规划、夹爪等待；先加高分辨率分段计时，再评估轨迹复用。不可从整秒事件推导毫秒级规划速度。',
                  '4. 优先消除重复采样、重复规划和无意义重试；历史会话还包含故障锁停与结束后的空闲，应单独记录。保持现有速度、抬升高度和夹爪等待，直到新的真机测量支持调整。', ''])
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('results_dir', type=Path)
    parser.add_argument('--json', action='store_true', help='Print structured intervals')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    paths = sorted(args.results_dir.glob('continuous_apple_grasp_*.json'))
    if not paths:
        parser.error('没有找到 continuous_apple_grasp_*.json')
    analysis = analyze_directory(args.results_dir)
    output = (json.dumps(analysis, ensure_ascii=False, indent=2, allow_nan=False)
              if args.json else markdown_report(**analysis))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output + '\n', encoding='utf-8')
    else:
        print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
