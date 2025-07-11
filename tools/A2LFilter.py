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
    lines = [line.rstrip() for line in lines]  # 去除尾部空白
    if lines and re.match(r"/\*\s*Auto-generated:.*\*/", lines[0]):
        lines = lines[1:]
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

    with open(path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(lines)

    if content_changed:
        print(f"[INFO] Generate: {path}")
    else:
        print(f"Contents already match: {path}")

    return content_changed


def parse_blocks(lines):
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

    compu_map = {
        "UBYTE": ("NO_COMPU_UBYTE", "%3.0"),
        "SBYTE": ("NO_COMPU_SBYTE", "%4.0"),
        "UWORD": ("NO_COMPU_UWORD", "%5.0"),
        "SWORD": ("NO_COMPU_SWORD", "%6.0"),
        "ULONG": ("NO_COMPU_ULONG", "%10.0"),
        "SLONG": ("NO_COMPU_SLONG", "%11.0"),
        "FLOAT32_IEEE": ("NO_COMPU_FLOAT32", "%8.6"),
    }

    skip_patterns = [
        re.compile(r".*\._\d+_\..*"),
        re.compile(r".*Table.*"),
        re.compile(r".*Coef.*"),
        re.compile(r".*husart0.*"),
        re.compile(r".*hcan0.*"),
        re.compile(r".*_lut*"),
        re.compile(r".*ccp.*"),
        re.compile(r".*GPIOD_InitStruct.*"),
    ]

    out_lines, idx = [], 0
    while idx < len(lines):
        line = lines[idx]
        stripped = line.strip()

        if stripped.startswith("/begin MODULE"):
            out_lines.append(line)
            out_lines.append("\n")
            for dtype, (mname, fmt) in compu_map.items():
                out_lines.append(
                    f'    /begin COMPU_METHOD {mname} "Identity {dtype}"\n')
                out_lines.append("      RAT_FUNC\n")
                out_lines.append(f'      "{fmt}"\n')
                out_lines.append('      ""\n')
                out_lines.append("      COEFFS 0 1 0 0 0 1\n")
                out_lines.append("    /end COMPU_METHOD\n")
                out_lines.append("\n")
            idx += 1
            continue

        block = next(((s, e, name) for (s, e, name) in mblocks if s == idx), None)
        if block:
            s, e, name = block
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

        out_lines.append(line)
        idx += 1

    ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    out_lines.insert(0, f"/* Auto-generated: {ts} */\n")

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
        # 比较变化并打印
    last_path = out_path + ".last"
    if os.path.exists(last_path):
        original_lines = _strip_dynamic_lines(read_file(last_path))
    else:
        original_lines = []

    original_blocks = parse_blocks(original_lines)
    original_dict = block_dict_raw(original_blocks, original_lines)

    final_stripped = _strip_dynamic_lines(final)
    generated_blocks = parse_blocks(final_stripped)
    generated_dict = block_dict_raw(generated_blocks, final_stripped)

    original_names = set(original_dict.keys())
    generated_names = set(generated_dict.keys())

    for name in sorted(generated_names - original_names):
        print(f"[ADDED] {name}")
    for name in sorted(original_names - generated_names):
        print(f"[REMOVED] {name}")
    address_changed_count = 0
    other_changed_count = 0

    for name in sorted(original_names & generated_names):
        old_block = original_dict[name]
        new_block = generated_dict[name]

        if old_block == new_block:
            continue

        changes = []
        for old_line, new_line in zip(old_block, new_block):
            old_line = old_line.strip()
            new_line = new_line.strip()
            if old_line == new_line:
                continue
            old_field = old_line.split(maxsplit=1)[0]
            new_field = new_line.split(maxsplit=1)[0]
            if old_field == new_field:
                changes.append((old_field, old_line[len(old_field):].strip(), new_line[len(new_field):].strip()))

        if not changes:
            continue

        printed_header = False
        for field, old_val, new_val in changes:
            if field == "ECU_ADDRESS":
                if (old_val == "0x0" and new_val != "0x0") or (old_val != "0x0" and new_val == "0x0"):
                    print(f"[CHANGED] {name}  ECU_ADDRESS {old_val} -> {new_val}")
                    address_changed_count += 1
                    printed_header = True
            else:
                if not printed_header:
                    print(f"[CHANGED] {name}  {field} {old_val} -> {new_val}")
                    printed_header = True
                else:
                    print(f"           {field} {old_val} -> {new_val}")
        if not printed_header:
            other_changed_count += 1

    if other_changed_count:
        print(f"[EFFECT] {other_changed_count} variables changed")
                    
    with open(last_path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(final)

    # ---------- 比较变化并打印 ---------- #
def block_dict_raw(blocks, lines):
    """构造 name -> list of lines(原始块内容)"""
    d = {}
    for kw, name, s, e in blocks:
        if kw in ("MEASUREMENT", "CHARACTERISTIC"):
            d[name] = [lines[i].rstrip() for i in range(s, e + 1)]
    return d

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Filt A2L, Replace NO_COMPU_METHOD and write COMPU_METHOD according to variable type.\n",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""Usage:
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
