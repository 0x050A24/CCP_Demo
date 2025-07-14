#!/usr/bin/env python3
import re
import sys
import os
import argparse
from datetime import datetime


def read_file(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()


def _strip_dynamic_lines(lines):
    # 去除首行时间戳
    lines = [line.rstrip() for line in lines]  # 去除所有尾部空白
    if lines and re.match(r"/\*\s*Auto-generated:.*\*/", lines[0]):
        lines = lines[1:]
    # 去除末尾所有空行
    while lines and lines[-1].strip() == "":
        lines.pop()
    return lines


def write_file(path, lines):
    new_lines_cleaned = _strip_dynamic_lines(lines)
    last_path = path + ".last"

    content_changed = True
    if os.path.exists(last_path):
        with open(last_path, "r", encoding="utf-8") as f:
            last_lines = f.readlines()
        last_lines_cleaned = _strip_dynamic_lines(last_lines)
        if last_lines_cleaned == new_lines_cleaned:
            content_changed = False

    # 不论如何都写入，保持文件带脚本改动
    with open(path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(lines)

    # 更新.last
    with open(last_path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(lines)

    if content_changed:
        print(f"[INFO] Generate: {path}")
    else:
        print(f"Contents already match: {path}")


def parse_blocks(lines):
    """返回所有 /begin.../end 块: (keyword, name, start, end)"""
    blocks, stack = [], []
    for i, ln in enumerate(lines):
        stripped = ln.strip()
        if stripped.startswith("/begin"):
            parts = stripped.split()
            if len(parts) >= 3:
                stack.append((parts[1], parts[2], i))
        elif stripped.startswith("/end") and stack:
            parts = stripped.split()
            if len(parts) >= 2:
                kw, name, start = stack.pop()
                if kw == parts[1]:
                    blocks.append((kw, name, start, i))
    return blocks


def filter_a2l(in_path, out_path):
    lines = read_file(in_path)
    blocks = parse_blocks(lines)
    mblocks = [
        (s, e, name)
        for (kw, name, s, e) in blocks
        if kw in ("MEASUREMENT", "CHARACTERISTIC")
    ]

    # 定义类型到 CompMethod 名称和 printf 格式映射
    compu_map = {
        "UBYTE": ("NO_COMPU_UBYTE", "%3.0"),
        "SBYTE": ("NO_COMPU_SBYTE", "%4.0"),
        "UWORD": ("NO_COMPU_UWORD", "%5.0"),
        "SWORD": ("NO_COMPU_SWORD", "%6.0"),
        "ULONG": ("NO_COMPU_ULONG", "%10.0"),
        "SLONG": ("NO_COMPU_SLONG", "%11.0"),
        "FLOAT32_IEEE": ("NO_COMPU_FLOAT32", "%8.6"),
    }
    # 跳过模式列表，便于后续扩展
    skip_patterns = [
        re.compile(r".*\._\d+_\..*"),  # 带 indexed 成员的变量
        re.compile(r".*Table.*"),
        re.compile(r".*Coef.*"),
        re.compile(r".*husart0.*"),
        re.compile(r".*hcan0.*"),
        re.compile(r".*_lut*"),
        re.compile(r".*ccp.*"),
        re.compile(r".*GPIOD_InitStruct.*"),
        # 添加更多需要跳过的正则，如:
        # re.compile(r'^temp_.*'),
    ]

    out_lines, idx = [], 0
    while idx < len(lines):
        line = lines[idx]
        stripped = line.strip()

        # 注入 CompMethods 定义到 MODULE 开始处，并添加空行
        if stripped.startswith("/begin MODULE"):
            out_lines.append(line)
            out_lines.append("\n")  # MODULE 与首个 COMPU_METHOD 之间空行
            for dtype, (mname, fmt) in compu_map.items():
                out_lines.append(
                    f'    /begin COMPU_METHOD {mname} "Identity {dtype}"\n'
                )
                out_lines.append("      RAT_FUNC\n")
                out_lines.append(f'      "{fmt}"\n')
                out_lines.append('      ""\n')
                out_lines.append("      COEFFS 0 1 0 0 0 1\n")
                out_lines.append(f"    /end COMPU_METHOD\n")
                out_lines.append("\n")  # 每个 COMPU_METHOD 之后空行
            idx += 1
            continue

        # MEASUREMENT/CHARACTERISTIC 处理
        block = next(((s, e, name) for (s, e, name) in mblocks if s == idx), None)
        if block:
            s, e, name = block
            # 检查是否匹配任何跳过模式
            if any(pat.match(name) for pat in skip_patterns):
                idx = e + 1
                continue
            out_lines.append("\n")
            for i in range(s, e + 1):
                m = re.match(
                    r"(\s*)(" + "|".join(compu_map.keys()) + r")\s+NO_COMPU_METHOD(.*)",
                    lines[i],
                )
                if m:
                    dtype = m.group(2)
                    rest = m.group(3)
                    mname = compu_map[dtype][0]
                    out_lines.append(f"{m.group(1)}{dtype} {mname}{rest}\n")
                else:
                    out_lines.append(lines[i])
            idx = e + 1
            continue

        # 其它行直接复制
        out_lines.append(line)
        idx += 1

    # 插入头时间戳
    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    out_lines.insert(0, f"/* Auto-generated: {ts} */\n")

    # 清理多余空行
    final, prev_empty = [], False
    for ln in out_lines:
        if not ln.strip():
            if not prev_empty:
                final.append(ln)
            prev_empty = True
        else:
            final.append(ln)
            prev_empty = False

    write_file(out_path, final)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Filt A2L, Replace NO_COMPU_METHOD and write COMPU_METHOD according to variable type.\n",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""Usage：
  filter_a2l.py -i input.a2l -o output.a2l""",
    )
    parser.add_argument(
        "-i", "--input", dest="input", required=True, help="Input A2L file path"
    )
    parser.add_argument(
        "-o", "--output", dest="output", required=True, help="Output A2L file path"
    )
    args = parser.parse_args()
    filter_a2l(args.input, args.output)
