#!/usr/bin/env python3
"""
A2L 文件过滤器工具

该工具用于处理和优化 A2L (ASAP2 Calibration Description Language) 文件
主要功能：
1. 移除不需要的变量和表格
2. 替换计算方法 (COMPU_METHOD)
3. 比较文件变化并生成报告
4. 去除多余的空行

A2L 文件是汽车标定工具中用于描述ECU内部变量的标准格式文件
"""

import re
import sys
import os
import argparse
from datetime import datetime
from typing import List, Tuple, Dict, Set, Optional


# ==============================================================================
# 文件 I/O 操作函数
# ==============================================================================

def read_file(path: str) -> List[str]:
    """
    读取文件内容并返回行列表
    
    Args:
        path: 文件路径
        
    Returns:
        文件内容的行列表
    """
    with open(path, "r", encoding="utf-8") as f:
        return f.readlines()


def _strip_dynamic_lines(lines: List[str]) -> List[str]:
    """
    移除动态生成的内容（如时间戳）和尾部空行
    
    这个函数用于标准化文件内容，以便进行有意义的比较
    
    Args:
        lines: 原始文件行列表
        
    Returns:
        清理后的行列表（移除了时间戳和尾部空行）
    """
    # 去除每行尾部的空白字符
    lines = [line.rstrip() for line in lines]
    
    # 移除首行的自动生成时间戳注释
    if lines and re.match(r"/\*\s*Auto-generated:.*\*/", lines[0]):
        lines = lines[1:]
    
    # 移除文件末尾的空行
    while lines and lines[-1].strip() == "":
        lines.pop()
        
    return lines


def write_file(path: str, lines: List[str]) -> bool:
    """
    写入文件并检查内容是否发生变化
    
    该函数会与上次生成的文件进行比较，只有在内容确实发生变化时才报告更新
    
    Args:
        path: 输出文件路径
        lines: 要写入的内容行列表
        
    Returns:
        True 如果内容发生了变化，False 如果内容相同
    """
    new_lines_cleaned = _strip_dynamic_lines(lines)
    last_path = path + ".last"

    content_changed = True
    
    # 检查是否存在上次生成的文件，如果存在则比较内容
    if os.path.exists(last_path):
        with open(last_path, "r", encoding="utf-8") as f:
            last_lines = f.readlines()
        last_lines_cleaned = _strip_dynamic_lines(last_lines)
        
        # 如果清理后的内容相同，则认为没有实质性变化
        if last_lines_cleaned == new_lines_cleaned:
            content_changed = False

    # 写入新文件
    with open(path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(lines)

    # 根据变化情况输出不同的消息
    if content_changed:
        print(f"[INFO] Generate: {path}")
    else:
        print(f"Contents already match: {path}")

    return content_changed


# ==============================================================================
# A2L 文件解析函数
# ==============================================================================

def parse_blocks(lines: List[str]) -> List[Tuple[str, str, int, int]]:
    """
    解析 A2L 文件中的块结构
    
    A2L 文件使用 /begin...../end 语法定义块结构
    例如：/begin MEASUREMENT var_name .... /end MEASUREMENT
    
    Args:
        lines: A2L 文件的所有行
        
    Returns:
        块信息列表，每个元素为 (关键字, 名称, 开始行号, 结束行号)
        例如：('MEASUREMENT', 'speed', 10, 15)
    """
    blocks = []
    stack = []  # 用于处理嵌套块的栈
    
    for i, ln in enumerate(lines):
        stripped = ln.strip()
        
        # 处理块开始标记 /begin
        if stripped.startswith("/begin"):
            parts = stripped.split()
            if len(parts) >= 3:
                # parts[1] 是关键字（如 MEASUREMENT），parts[2] 是名称
                stack.append((parts[1], parts[2], i))
                
        # 处理块结束标记 /end
        elif stripped.startswith("/end") and stack:
            parts = stripped.split()
            if len(parts) >= 2:
                # 确保 /end 与对应的 /begin 匹配
                kw, name, start = stack.pop()
                if kw == parts[1]:
                    blocks.append((kw, name, start, i))
                    
    return blocks


def block_dict_raw(blocks: List[Tuple[str, str, int, int]], lines: List[str]) -> Dict[str, List[str]]:
    """
    构造变量名到原始块内容的字典映射
    
    Args:
        blocks: 解析得到的块信息列表
        lines: 原始文件行列表
        
    Returns:
        字典，键为变量名，值为该变量的完整块内容（行列表）
    """
    result = {}
    for kw, name, start, end in blocks:
        # 只处理测量变量和特征变量
        if kw in ("MEASUREMENT", "CHARACTERISTIC"):
            result[name] = [lines[i].rstrip() for i in range(start, end + 1)]
    return result


# ==============================================================================
# A2L 文件过滤和处理主函数
# ==============================================================================

def filter_a2l(in_path: str, out_path: str) -> None:
    """
    A2L 文件过滤主函数
    
    该函数执行以下操作：
    1. 读取输入的 A2L 文件
    2. 解析文件结构，识别测量变量和特征变量
    3. 根据预定义规则过滤不需要的变量
    4. 生成标准化的计算方法 (COMPU_METHOD)
    5. 输出处理后的 A2L 文件
    6. 比较并报告变化
    
    Args:
        in_path: 输入 A2L 文件路径
        out_path: 输出 A2L 文件路径
    """
    # 读取输入文件
    lines = read_file(in_path)
    
    # 解析 A2L 文件的块结构
    blocks = parse_blocks(lines)
    
    # 提取测量变量和特征变量的块信息
    measurement_blocks = [
        (start, end, name)
        for (keyword, name, start, end) in blocks
        if keyword in ("MEASUREMENT", "CHARACTERISTIC")
    ]

    # 定义数据类型到计算方法的映射
    # 这些计算方法用于定义如何显示和转换变量值
    compu_method_mapping = {
        "UBYTE": ("NO_COMPU_UBYTE", "%3.0"),      # 无符号8位整数
        "SBYTE": ("NO_COMPU_SBYTE", "%4.0"),      # 有符号8位整数
        "UWORD": ("NO_COMPU_UWORD", "%5.0"),      # 无符号16位整数
        "SWORD": ("NO_COMPU_SWORD", "%6.0"),      # 有符号16位整数
        "ULONG": ("NO_COMPU_ULONG", "%10.0"),     # 无符号32位整数
        "SLONG": ("NO_COMPU_SLONG", "%11.0"),     # 有符号32位整数
        "FLOAT32_IEEE": ("NO_COMPU_FLOAT32", "%8.6"),  # 32位浮点数
    }

    # 定义要跳过的变量名模式
    # 这些模式用于过滤掉不需要在标定工具中显示的内部变量
    skip_variable_patterns = [
        re.compile(r".*\._\d+_\..*"),        # 编译器生成的临时变量
        re.compile(r".*Table.*"),            # 各种表格变量
        re.compile(r".*Coef.*"),             # 系数变量
        re.compile(r".*husart0.*"),          # UART硬件相关变量
        re.compile(r".*hcan0.*"),            # CAN硬件相关变量
        re.compile(r".*_lut*"),              # 查找表变量
        re.compile(r".*ccp.*"),              # CCP协议相关变量
        re.compile(r".*GPIOD_InitStruct.*"), # GPIO初始化结构体
    ]

    # 开始处理文件内容
    output_lines = []
    current_index = 0
    
    while current_index < len(lines):
        current_line = lines[current_index]
        stripped_line = current_line.strip()

        # 处理 MODULE 块的开始 - 在此处插入计算方法定义
        if stripped_line.startswith("/begin MODULE"):
            output_lines.append(current_line)
            output_lines.append("\n")
            
            # 为每种数据类型生成对应的计算方法定义
            for data_type, (method_name, format_string) in compu_method_mapping.items():
                output_lines.extend([
                    f'    /begin COMPU_METHOD {method_name} "Identity {data_type}"\n',
                    "      RAT_FUNC\n",
                    f'      "{format_string}"\n',
                    '      ""\n',
                    "      COEFFS 0 1 0 0 0 1\n",  # 线性转换系数：y = 0 + 1*x
                    "    /end COMPU_METHOD\n",
                    "\n"
                ])
            current_index += 1
            continue

        # 检查当前行是否是测量变量或特征变量块的开始
        current_block = next(
            ((start, end, name) for (start, end, name) in measurement_blocks if start == current_index), 
            None
        )
        
        if current_block:
            start_line, end_line, variable_name = current_block
            
            # 检查是否应该跳过这个变量
            if any(pattern.match(variable_name) for pattern in skip_variable_patterns):
                current_index = end_line + 1  # 跳过整个块
                continue
            
            # 处理变量块：替换计算方法引用
            output_lines.append("\n")
            for line_idx in range(start_line, end_line + 1):
                line_content = lines[line_idx]
                
                # 查找并替换 NO_COMPU_METHOD 为对应的计算方法
                compu_match = re.match(
                    r"(\s*)(" + "|".join(compu_method_mapping.keys()) + r")\s+NO_COMPU_METHOD(.*)",
                    line_content,
                )
                
                if compu_match:
                    # 提取匹配的数据类型和其他部分
                    indentation = compu_match.group(1)
                    data_type = compu_match.group(2)
                    remaining_content = compu_match.group(3)
                    method_name = compu_method_mapping[data_type][0]
                    
                    # 构造新的行内容
                    new_line = f"{indentation}{data_type} {method_name}{remaining_content}\n"
                    output_lines.append(new_line)
                else:
                    # 如果不需要替换，直接添加原行
                    output_lines.append(line_content)
                    
            current_index = end_line + 1
            continue

        # 对于其他行，直接添加到输出
        output_lines.append(current_line)
        current_index += 1

    # 添加时间戳注释到文件开头
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    output_lines.insert(0, f"/* Auto-generated: {timestamp} */\n")

    # 清理多余的空行（连续的空行合并为单个空行）
    final_lines = []
    previous_line_empty = False
    
    for line in output_lines:
        current_line_empty = not line.strip()
        
        if current_line_empty:
            # 只有在前一行不是空行时才添加空行
            if not previous_line_empty:
                final_lines.append(line)
            previous_line_empty = True
        else:
            final_lines.append(line)
            previous_line_empty = False

    # 写入处理后的文件
    write_file(out_path, final_lines)
    
    # 比较文件变化并生成报告
    _compare_and_report_changes(out_path, final_lines)
# ==============================================================================
# 文件变化比较和报告函数
# ==============================================================================

def _compare_and_report_changes(out_path: str, final_lines: List[str]) -> None:
    """
    比较新生成的文件与上次生成的文件，报告变化情况
    
    Args:
        out_path: 输出文件路径
        final_lines: 新生成的文件内容
    """
    last_path = out_path + ".last"
    
    # 读取上次生成的文件（如果存在）
    if os.path.exists(last_path):
        original_lines = _strip_dynamic_lines(read_file(last_path))
    else:
        original_lines = []

    # 解析两个版本的文件结构
    original_blocks = parse_blocks(original_lines)
    original_dict = block_dict_raw(original_blocks, original_lines)

    final_stripped = _strip_dynamic_lines(final_lines)
    generated_blocks = parse_blocks(final_stripped)
    generated_dict = block_dict_raw(generated_blocks, final_stripped)

    # 获取变量名集合
    original_names = set(original_dict.keys())
    generated_names = set(generated_dict.keys())

    # 报告新增的变量
    for name in sorted(generated_names - original_names):
        print(f"[ADDED] {name}")
        
    # 报告删除的变量
    for name in sorted(original_names - generated_names):
        print(f"[REMOVED] {name}")
        
    # 分析变量的变化
    address_changed_count = 0
    other_changed_count = 0

    # 检查共同存在的变量的变化
    for name in sorted(original_names & generated_names):
        old_block = original_dict[name]
        new_block = generated_dict[name]

        # 如果块内容完全相同，跳过
        if old_block == new_block:
            continue

        # 分析具体的变化
        changes = []
        for old_line, new_line in zip(old_block, new_block):
            old_line = old_line.strip()
            new_line = new_line.strip()
            
            if old_line == new_line:
                continue
                
            # 提取字段名和值
            old_parts = old_line.split(maxsplit=1)
            new_parts = new_line.split(maxsplit=1)
            
            if len(old_parts) >= 1 and len(new_parts) >= 1:
                old_field = old_parts[0]
                new_field = new_parts[0]
                
                if old_field == new_field:
                    old_value = old_parts[1] if len(old_parts) > 1 else ""
                    new_value = new_parts[1] if len(new_parts) > 1 else ""
                    changes.append((old_field, old_value, new_value))

        if not changes:
            continue

        # 报告变化，特别关注地址变化
        printed_header = False
        for field, old_val, new_val in changes:
            if field == "ECU_ADDRESS":
                # ECU地址变化通常表示变量在内存中的位置发生了变化
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

    # 输出变化统计
    if other_changed_count:
        print(f"[EFFECT] {other_changed_count} variables changed")
    
    # 保存当前版本作为下次比较的基准
    with open(last_path, "w", encoding="utf-8", newline='\n') as f:
        f.writelines(final_lines)

# ==============================================================================
# 命令行接口
# ==============================================================================

if __name__ == "__main__":
    # 创建命令行参数解析器
    parser = argparse.ArgumentParser(
        description="""A2L 文件过滤工具
        
该工具用于处理和优化 A2L 标定文件：
• 移除不需要的内部变量和临时变量
• 替换 NO_COMPU_METHOD 为标准化的计算方法
• 自动生成对应数据类型的计算方法定义
• 比较文件变化并生成详细报告
• 优化文件格式，移除多余空行

A2L 文件是汽车ECU标定中使用的标准描述文件格式""",
        formatter_class=argparse.RawTextHelpFormatter,
        epilog="""使用示例:
  python A2LFilter.py -i input.a2l -o output.a2l
  python A2LFilter.py --input raw_file.a2l --output filtered_file.a2l""",
    )
    
    # 定义命令行参数
    parser.add_argument(
        "-i", "--input", 
        dest="input", 
        required=True, 
        help="输入的 A2L 文件路径"
    )
    parser.add_argument(
        "-o", "--output", 
        dest="output", 
        required=True, 
        help="输出的 A2L 文件路径"
    )
    
    # 解析命令行参数
    args = parser.parse_args()
    
    # 检查输入文件是否存在
    if not os.path.exists(args.input):
        print(f"错误: 输入文件 '{args.input}' 不存在")
        sys.exit(1)
    
    # 确保输出目录存在
    output_dir = os.path.dirname(args.output)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 执行 A2L 文件过滤
    try:
        print(f"开始处理 A2L 文件: {args.input}")
        filter_a2l(args.input, args.output)
        print(f"处理完成，输出文件: {args.output}")
    except Exception as e:
        print(f"处理过程中发生错误: {e}")
        sys.exit(1)
