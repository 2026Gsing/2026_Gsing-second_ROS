import os, json, time
import cv2 as cv
import numpy as np
from collections import OrderedDict
from PIL import Image, ImageDraw, ImageFont

def default_config_path():
    try:
        base = os.path.dirname(os.path.abspath(__file__))  # 脚本所在目录
        if not base:
            base = os.getcwd()
    except NameError:
        base = os.getcwd()
    return os.path.join(base, "camera_four_colors_config.json")

CONFIG_PATH = os.environ.get("CFC_CONFIG", default_config_path())

def _pick_font():
    candidates = [
        "C:/Windows/Fonts/msyh.ttc",    # 微软雅黑（Win）
        "C:/Windows/Fonts/simhei.ttf",  # 黑体（Win）
        "/System/Library/Fonts/PingFang.ttc",  # macOS
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",  # Linux
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None

_CN_FONT_PATH = _pick_font()

def put_text_cn(img_bgr, text, pos, font_size=22, color=(255,255,255)):
    if not text:
        return img_bgr
    if _CN_FONT_PATH is None:
        cv.putText(img_bgr, text, pos, cv.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv.LINE_AA)
        return img_bgr
    img_pil = Image.fromarray(cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(img_pil)
    font = ImageFont.truetype(_CN_FONT_PATH, font_size)
    rgb = (color[2], color[1], color[0])  # BGR -> RGB
    draw.text(pos, text, font=font, fill=rgb)
    return cv.cvtColor(np.array(img_pil), cv.COLOR_RGB2BGR)

# ---------------------- 默认四类 HSV 范围（首次运行会用它们） ----------------------
DEFAULT_CLASS_RANGES = OrderedDict({
    "食品(绿)": [((40, 60, 60), (70, 255, 255))],
    "工具(灰)": [((0, 0, 80), (180, 50, 190))],  # 低饱和+中亮度
    "药品(红)": [((0, 120, 80), (10, 255, 255)),
              ((170, 120, 80), (180, 255, 255))],
    "仪器(蓝)": [((100, 100, 70), (125, 255, 255))],
})

CLASS_KEYS = OrderedDict({
    '1': "食品(绿)",
    '2': "工具(灰)",
    '3': "药品(红)",
    '4': "仪器(蓝)",
})

# ---------------------- OpenCV UI ----------------------
def nothing(x): pass

def make_windows():
    cv.namedWindow("Frame", cv.WINDOW_NORMAL)
    cv.namedWindow("MaskBest", cv.WINDOW_NORMAL)
    cv.namedWindow("MaskEdit", cv.WINDOW_NORMAL)
    cv.namedWindow("Controls", cv.WINDOW_NORMAL)

    # HSV 段1
    cv.createTrackbar("H1_min", "Controls", 40, 180, nothing)
    cv.createTrackbar("H1_max", "Controls", 70, 180, nothing)
    cv.createTrackbar("S_min",  "Controls", 60, 255, nothing)
    cv.createTrackbar("S_max",  "Controls", 255,255, nothing)
    cv.createTrackbar("V_min",  "Controls", 60, 255, nothing)
    cv.createTrackbar("V_max",  "Controls", 255,255, nothing)
    # 段2（可选，用于红色）
    cv.createTrackbar("Use_H2(0/1)", "Controls", 0, 1, nothing)
    cv.createTrackbar("H2_min", "Controls", 170, 180, nothing)
    cv.createTrackbar("H2_max", "Controls", 180, 180, nothing)
    # 形态学 & 边缘 & ROI 面积阈值
    cv.createTrackbar("Open_iter",  "Controls", 1, 5, nothing)
    cv.createTrackbar("Close_iter", "Controls", 1, 5, nothing)
    cv.createTrackbar("Kernel(odd)","Controls", 5, 15, nothing)   # 3/5/7...
    cv.createTrackbar("Canny_lo",   "Controls", 80, 400, nothing)
    cv.createTrackbar("Canny_hi",   "Controls", 160,400, nothing)
    # 注意：Min_Ratio(%) 保留但不参与判定逻辑（Top-1 直判）
    cv.createTrackbar("Min_Ratio(%)","Controls", 25, 100, nothing)
    cv.createTrackbar("Min_ROI_Area(%)","Controls", 3, 100, nothing)

def read_controls():
    g = cv.getTrackbarPos
    params = {
        "H1_min": g("H1_min","Controls"), "H1_max": g("H1_max","Controls"),
        "S_min":  g("S_min","Controls"),  "S_max":  g("S_max","Controls"),
        "V_min":  g("V_min","Controls"),  "V_max":  g("V_max","Controls"),
        "use_h2": g("Use_H2(0/1)","Controls")==1,
        "H2_min": g("H2_min","Controls"), "H2_max": g("H2_max","Controls"),
        "open_iter":  g("Open_iter","Controls"),
        "close_iter": g("Close_iter","Controls"),
        "kernel": max(3, g("Kernel(odd)","Controls") | 1),  # 强制奇数且≥3
        "canny_lo": g("Canny_lo","Controls"),
        "canny_hi": g("Canny_hi","Controls"),
        "min_ratio": g("Min_Ratio(%)","Controls")/100.0,      # 已不用于判定
        "min_roi_pct": g("Min_ROI_Area(%)","Controls")/100.0,
    }
    return params

def set_controls_from_ranges(ranges):
    (h1_lo, s_lo, v_lo), (h1_hi, s_hi, v_hi) = ranges[0]
    cv.setTrackbarPos("H1_min","Controls", int(h1_lo))
    cv.setTrackbarPos("H1_max","Controls", int(h1_hi))
    cv.setTrackbarPos("S_min","Controls",  int(s_lo))
    cv.setTrackbarPos("S_max","Controls",  int(s_hi))
    cv.setTrackbarPos("V_min","Controls",  int(v_lo))
    cv.setTrackbarPos("V_max","Controls",  int(v_hi))
    if len(ranges) > 1:
        (h2_lo, _, _), (h2_hi, _, _) = ranges[1]
        cv.setTrackbarPos("Use_H2(0/1)","Controls", 1)
        cv.setTrackbarPos("H2_min","Controls", int(h2_lo))
        cv.setTrackbarPos("H2_max","Controls", int(h2_hi))
    else:
        cv.setTrackbarPos("Use_H2(0/1)","Controls", 0)

def set_controls_from_dict(ctrl):
    # 恢复全局滑块
    def _set(name, val):
        try: cv.setTrackbarPos(name, "Controls", int(val))
        except: pass
    _set("H1_min", ctrl.get("H1_min", 40))
    _set("H1_max", ctrl.get("H1_max", 70))
    _set("S_min",  ctrl.get("S_min", 60))
    _set("S_max",  ctrl.get("S_max", 255))
    _set("V_min",  ctrl.get("V_min", 60))
    _set("V_max",  ctrl.get("V_max", 255))
    _set("Use_H2(0/1)", 1 if ctrl.get("use_h2", False) else 0)
    _set("H2_min", ctrl.get("H2_min", 170))
    _set("H2_max", ctrl.get("H2_max", 180))
    _set("Open_iter",  ctrl.get("open_iter", 1))
    _set("Close_iter", ctrl.get("close_iter", 1))
    _set("Kernel(odd)",ctrl.get("kernel", 5))
    _set("Canny_lo",   ctrl.get("canny_lo", 80))
    _set("Canny_hi",   ctrl.get("canny_hi", 160))
    _set("Min_Ratio(%)",    int(round(ctrl.get("min_ratio", 0.25)*100)))
    _set("Min_ROI_Area(%)", int(round(ctrl.get("min_roi_pct", 0.03)*100)))

def ranges_from_controls(ctrl):
    ranges = [((ctrl["H1_min"], ctrl["S_min"], ctrl["V_min"]),
               (ctrl["H1_max"], ctrl["S_max"], ctrl["V_max"]))]
    if ctrl["use_h2"]:
        ranges.append(((ctrl["H2_min"], ctrl["S_min"], ctrl["V_min"]),
                       (ctrl["H2_max"], ctrl["S_max"], ctrl["V_max"])))
    return ranges

# ---------------------- 颜色掩膜 / ROI ----------------------
def color_mask_from_ranges(hsv, ranges, kernel, open_iter, close_iter):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in ranges:
        lo = np.array(lo, np.uint8)
        hi = np.array(hi, np.uint8)
        mask = cv.bitwise_or(mask, cv.inRange(hsv, lo, hi))
    k = np.ones((kernel, kernel), np.uint8)
    if open_iter > 0:
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k, iterations=open_iter)
    if close_iter > 0:
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k, iterations=close_iter)
    return mask

def find_largest_contour(frame, canny_lo, canny_hi):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5,5), 0)
    edges = cv.Canny(gray, canny_lo, canny_hi)
    edges = cv.dilate(edges, np.ones((3,3), np.uint8), iterations=1)
    res = cv.findContours(edges, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = res[0] if len(res) == 2 else res[1]
    if not cnts: return None
    return max(cnts, key=cv.contourArea)

def rotated_rect_mask(shape, contour):
    rect = cv.minAreaRect(contour)   # (center, (w,h), angle)
    box  = cv.boxPoints(rect)
    box  = np.rint(box).astype(np.int32)  # NumPy 2.x 兼容
    mask = np.zeros(shape[:2], dtype=np.uint8)
    cv.fillPoly(mask, [box], 255)
    return mask, box, rect

# ---------------------- 配置 读/写 ----------------------
def _ranges_to_jsonable(class_ranges):
    out = {}
    for k, segs in class_ranges.items():
        out[k] = [ [list(lo), list(hi)] for (lo,hi) in segs ]
    return out

def _ranges_from_json(dct, fallback):
    out = OrderedDict()
    for k in fallback.keys():
        segs = dct.get(k, None)
        if not segs:
            out[k] = fallback[k]
            continue
        norm = []
        try:
            for pair in segs:
                lo, hi = pair
                norm.append( (tuple(map(int,lo)), tuple(map(int,hi))) )
            out[k] = norm
        except:
            out[k] = fallback[k]
    return out

def load_config():
    if not os.path.exists(CONFIG_PATH):
        return {
            "class_ranges": DEFAULT_CLASS_RANGES.copy(),
            "controls": {},
            "current_edit_class": list(DEFAULT_CLASS_RANGES.keys())[0]
        }
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        class_ranges = _ranges_from_json(data.get("class_ranges", {}), DEFAULT_CLASS_RANGES)
        controls = data.get("controls", {})
        cur = data.get("current_edit_class", list(class_ranges.keys())[0])
        if cur not in class_ranges:
            cur = list(class_ranges.keys())[0]
        return {"class_ranges": class_ranges, "controls": controls, "current_edit_class": cur}
    except Exception as e:
        print("加载配置失败，使用默认配置：", e)
        return {
            "class_ranges": DEFAULT_CLASS_RANGES.copy(),
            "controls": {},
            "current_edit_class": list(DEFAULT_CLASS_RANGES.keys())[0]
        }

def save_config(class_ranges, controls, current_edit_class):
    try:
        data = {
            "version": 1,
            "class_ranges": _ranges_to_jsonable(class_ranges),
            "controls": controls,
            "current_edit_class": current_edit_class
        }
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        print("保存配置失败：", e)

# ---------------------- 主程序 ----------------------
def main():
    cfg = load_config()
    class_ranges = cfg["class_ranges"]
    current_edit_class = cfg["current_edit_class"]

    make_windows()

    # 恢复全局滑块（若有）
    if cfg["controls"]:
        set_controls_from_dict(cfg["controls"])
    else:
        set_controls_from_ranges(class_ranges[current_edit_class])

    # 再保证把“当前编辑类别”的范围写入滑块
    set_controls_from_ranges(class_ranges[current_edit_class])

    cap = cv.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    last_save_ts = 0
    last_saved_controls = None
    last_saved_ranges = _ranges_to_jsonable(class_ranges)  # 用于变更检测

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读取帧失败")
            break

        h, w = frame.shape[:2]
        ctrl = read_controls()

        # --- 自动同步：把滑块实时写入“当前编辑类别”的范围 ---
        new_ranges = ranges_from_controls(ctrl)
        def _norm(rs): return [ [list(lo), list(hi)] for (lo,hi) in rs ]
        if _norm(new_ranges) != _ranges_to_jsonable({ "x": class_ranges[current_edit_class] })["x"]:
            class_ranges[current_edit_class] = new_ranges

        # 1) 找 ROI
        cnt = find_largest_contour(frame, ctrl["canny_lo"], ctrl["canny_hi"])
        roi_mask = np.zeros((h,w), np.uint8)
        roi_area = 0
        info_roi = "ROI: 无"
        if cnt is not None:
            roi_mask, box, rect = rotated_rect_mask(frame.shape, cnt)
            roi_area = int(np.count_nonzero(roi_mask))
            min_roi_area = int(ctrl["min_roi_pct"] * h * w)
            cv.drawContours(frame, [box], 0, (0,255,255), 2)
            if roi_area < max(50, min_roi_area):
                info_roi = f"ROI 太小: {roi_area} (< {min_roi_area})"
            else:
                info_roi = f"ROI 面积: {roi_area} ({roi_area/(h*w):.2%})"

        # 2) 统计占比 & Top-1 判定
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        ratios = []

        if roi_area > 0:
            for cls, ranges in class_ranges.items():
                mask = color_mask_from_ranges(
                    hsv, ranges, ctrl["kernel"], ctrl["open_iter"], ctrl["close_iter"]
                )
                mask_roi = cv.bitwise_and(mask, roi_mask)
                ratio = np.count_nonzero(mask_roi) / float(roi_area)
                ratios.append((cls, ratio, mask_roi))

            # ---- Top-1 直判：谁占比最高就判谁（不再看 Min_Ratio）----
            best_label, best_ratio, mask_best = max(ratios, key=lambda x: x[1])
            label_to_show = best_label
            cv.imshow("MaskBest", mask_best)

            # 编辑预览（当前滑块）
            mask_edit = color_mask_from_ranges(
                hsv, new_ranges, ctrl["kernel"], ctrl["open_iter"], ctrl["close_iter"]
            )
            cv.imshow("MaskEdit", cv.bitwise_and(mask_edit, roi_mask))

            frame = put_text_cn(frame, f"类别: {label_to_show}   占比: {best_ratio:.2%}",
                                (10, 28), font_size=24, color=(0,255,0))
        else:
            # 无 ROI 时也显示编辑掩膜方便调参
            mask_edit = color_mask_from_ranges(
                hsv, new_ranges, ctrl["kernel"], ctrl["open_iter"], ctrl["close_iter"]
            )
            cv.imshow("MaskBest", np.zeros((h,w), dtype=np.uint8))
            cv.imshow("MaskEdit", mask_edit)
            frame = put_text_cn(frame, "类别: 未知（未找到 ROI）", (10,28), font_size=24, color=(0,0,255))

        # 左上角信息
        frame = put_text_cn(frame, info_roi, (10,60), font_size=20, color=(255,255,0))
        frame = put_text_cn(frame, f"正在编辑: {current_edit_class}  [1-4 切换, s 保存]",
                            (10,86), font_size=20, color=(200,200,200))

        # 右上角显示各类占比
        if roi_area > 0 and ratios:
            y = 22
            for name, r, _ in ratios[:4]:
                frame = put_text_cn(frame, f"{name}: {r:.1%}", (w-260, y), font_size=20, color=(220,220,220))
                y += 22

        cv.imshow("Frame", frame)

        # ---- 自动保存（~1 秒一次）----
        now = time.time()
        controls_for_save = ctrl.copy()
        changed = False
        if last_saved_controls != controls_for_save:
            changed = True
        if last_saved_ranges != _ranges_to_jsonable(class_ranges):
            changed = True
        if changed and (now - last_save_ts) > 1.0:
            save_config(class_ranges, controls_for_save, current_edit_class)
            last_save_ts = now
            last_saved_controls = controls_for_save
            last_saved_ranges = _ranges_to_jsonable(class_ranges)

        # ---- 键盘交互 ----
        key = cv.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            break

        if key in [ord('1'), ord('2'), ord('3'), ord('4')]:
            slot = chr(key)
            current_edit_class = CLASS_KEYS[slot]
            set_controls_from_ranges(class_ranges[current_edit_class])
            print(f"[Load] {current_edit_class} ranges -> sliders")
            save_config(class_ranges, controls_for_save, current_edit_class)
            last_saved_ranges = _ranges_to_jsonable(class_ranges)

        if key == ord('s'):
            save_config(class_ranges, controls_for_save, current_edit_class)
            last_saved_ranges = _ranges_to_jsonable(class_ranges)
            last_saved_controls = controls_for_save
            last_save_ts = time.time()
            print("[Save] 手动保存完成")

        if key == ord('p'):
            print("当前四类范围：")
            for k, v in class_ranges.items():
                print("  ", k, ":", v)

    # 退出前再保存一次
    try:
        save_config(class_ranges, read_controls(), current_edit_class)
    except Exception:
        pass

    cap.release()
    cv.destroyAllWindows()

if __name__ == "__main__":
    main()
