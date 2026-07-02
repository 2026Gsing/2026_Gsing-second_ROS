from ultralytics import YOLO
import cv2
import numpy as np
import re
import sys
import os
from pathlib import Path

# ===================== 系统编码修复（Windows无问号核心） =====================
if sys.platform == "win32":
    os.environ['OPENCV_IO_ENCODING'] = 'UTF-8'
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    cv2.setNumThreads(0)
    cv2.ocl.setUseOpenCL(False)

# ===================== 模型与类别配置 =====================
_HERE = Path(__file__).resolve().parent
model = YOLO(str(_HERE / "../weights/math12.pt"))
# 你的类别列表：无等号，仅0-9、+、-、x、÷、(、)
class_names = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '+', '-', 'x', '÷', '(', ')'
]

# ===================== 符号映射表（解决÷显示问号，替换为/） =====================
symbol_map = {
    '÷': '/',  # 显示为/，避免问号
    'x': '*',  # 保持x→*的转换，计算用
    '+': '+',
    '-': '-',
    '(': '(',
    ')': ')',
    '0':'0','1':'1','2':'2','3':'3','4':'4','5':'5','6':'6','7':'7','8':'8','9':'9'
}

# ===================== 摄像头初始化 =====================
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
if not cap.isOpened():
    print("Error: Cannot open camera!")
    exit()
print("Camera started, press 'q' to exit...")

# ===================== 绘制参数（放大标注+左上角字体） =====================
box_color = (0, 255, 0)
box_thickness = 2

# 识别框标注（放大配置）
label_font = cv2.FONT_HERSHEY_PLAIN
label_font_scale = 3
label_font_thickness = 3
label_color = (255, 255, 255)

# 左上角算式/结果（放大配置）
result_font = cv2.FONT_HERSHEY_PLAIN
result_font_scale = 4
result_font_thickness = 4
result_color = (0, 0, 255)
result_pos = (30, 60)
ans_pos = (30, 110)

# ===================== 核心工具函数（修复÷显示问号） =====================
def extract_and_sort_symbols(boxes):
    """提取符号并排序，同时替换÷为/用于显示"""
    symbol_list = []
    display_symbol_list = []  # 用于显示的符号列表（÷→/）
    for box in result.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
        if (x2 - x1) <= 0 or (y2 - y1) <= 0:
            continue
        cls_idx = int(box.cls[0].cpu().numpy())
        conf = float(box.conf[0].cpu().numpy())
        if conf >= 0.5 and 0 <= cls_idx < len(class_names):
            cls_name = class_names[cls_idx]
            # 替换÷为/，用于显示（避免问号）
            display_name = symbol_map[cls_name]
            symbol_list.append((cls_name, x1))
            display_symbol_list.append((display_name, x1))
    # 按x轴排序
    symbol_list.sort(key=lambda x: x[1])
    display_symbol_list.sort(key=lambda x: x[1])
    # 返回原始符号（计算用）和显示符号（无问号）
    return [s[0] for s in symbol_list], [d[0] for d in display_symbol_list]

def build_equation(symbols):
    """拼接算式（计算用，x→*、÷→/）"""
    if not symbols:
        return ""
    equation = ""
    for s in symbols:
        if s == 'x':
            equation += '*'
        elif s == '÷':
            equation += '/'
        else:
            equation += s
    return equation

def build_display_equation(display_symbols):
    """拼接显示用算式（÷→/，无问号）"""
    if not display_symbols:
        return ""
    display_equation = ""
    for s in display_symbols:
        display_equation += s
    return display_equation

def is_safe_equation(equation):
    pattern = r"^[\d\+\-\*\/\(\)]+$"
    has_digit = any(c.isdigit() for c in equation)
    return bool(re.match(pattern, equation)) and has_digit

def safe_calculate(equation):
    if not equation:
        return "No equation"
    if not is_safe_equation(equation):
        return "Invalid equation"
    try:
        result = eval(equation)
        if isinstance(result, float):
            return f"{result:.2f}" if not result.is_integer() else f"{int(result)}"
        else:
            return f"{result}"
    except ZeroDivisionError:
        return "Div by 0"
    except SyntaxError:
        return "Syntax error"
    except Exception as e:
        print(f"计算异常详情：{str(e)}")
        return f"Calc err: {str(e)[:10]}"

# ===================== 主循环（修复÷显示问号） =====================
while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read frame!")
        break

    results = model(
        source=frame,
        imgsz=960,
        device=0,
        conf=0.5,
        iou=0.5,
        verbose=False,
        max_det=20,
        agnostic_nms=True
    )

    result = results[0]
    # 绘制检测框+替换后的标注（÷→/，无问号）
    for box in result.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
        if (x2 - x1) <= 0 or (y2 - y1) <= 0:
            continue
        cls_idx = int(box.cls[0].cpu().numpy())
        conf = float(box.conf[0].cpu().numpy())
        cls_name = class_names[cls_idx] if 0 <= cls_idx < len(class_names) else "?"
        # 关键：替换÷为/用于显示
        display_name = symbol_map.get(cls_name, "?")
        label_text = f"{display_name} {conf:.2f}"

        cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, box_thickness)
        # 放大后的标注位置
        cv2.putText(frame, label_text, (x1+5, y1-5),
                    label_font, label_font_scale, label_color, label_font_thickness)

    # 提取原始符号（计算用）和显示符号（无问号）
    symbols, display_symbols = extract_and_sort_symbols(result.boxes)
    # 拼接计算用算式和显示用算式
    equation = build_equation(symbols)
    display_equation = build_display_equation(display_symbols)
    # 计算结果
    answer = safe_calculate(equation)
    
    # 绘制左上角显示用算式（无问号）和结果
    cv2.putText(frame, f"Equation: {display_equation if display_equation else 'None'}",
                result_pos, result_font, result_font_scale, result_color, result_font_thickness)
    cv2.putText(frame, f"Result: {answer}",
                ans_pos, result_font, result_font_scale, result_color, result_font_thickness)

    cv2.imshow("YOLOv8 Math Calc (No Equal Sign)", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("Camera closed, exit successfully!")