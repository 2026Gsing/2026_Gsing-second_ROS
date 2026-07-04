import argparse
import json
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

_SRC_DIR = Path(__file__).resolve().parent        # vision/src/
_PROJ_DIR = _SRC_DIR.parent                       # vision/
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

from slot_roi import (  # noqa: E402
    assign_slots_for_detections,
    draw_slot_rois,
    load_slots_roi_config,
    warn_frame_size_mismatch,
)

"""
推理脚本：基于 Ultralytics YOLOv8
- 默认加载训练输出 runs/train/exp/weights/best.pt
- 支持图片/文件夹/视频/摄像头（0=内置，1=USB外接）

示例（PowerShell）：
python src/predict_yolo.py --weights weights/yolov8s.pt --source 1 --show  # USB摄像头
python src/predict_yolo.py --weights weights/yolov8s.pt --source 0 --show  # 内置摄像头
"""

def parse_args():
    parser = argparse.ArgumentParser(description="Predict with YOLOv8 model")
    parser.add_argument("--weights", type=str, default="../weights/task3.pt", help="模型权重路径")
    parser.add_argument("--source", type=str, default="1", help="输入源：0=内置摄像头，1=USB摄像头，或文件路径")
    parser.add_argument("--img", type=int, default=640, help="输入尺寸")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    parser.add_argument("--device", type=str, default="0", help="设备，例如 '0' 或 'cpu'")
    parser.add_argument("--project", type=str, default="runs/predict", help="输出项目目录")
    parser.add_argument("--name", type=str, default="exp", help="实验名称")
    parser.add_argument("--show", action="store_true", help="是否显示窗口")
    parser.add_argument("--save_video", action="store_true", help="是否保存视频结果")
    parser.add_argument("--cam-width", type=int, default=1080, help="摄像头采集宽度（像素）")
    parser.add_argument("--cam-height", type=int, default=720, help="摄像头采集高度（像素）")
    parser.add_argument("--window-width", type=int, default=1080, help="显示窗口宽度（像素）")
    parser.add_argument("--window-height", type=int, default=720, help="显示窗口高度（像素）")
    parser.add_argument(
        "--roi-config",
        type=str,
        default="../config/slots_roi.json",
        help="槽位 ROI 配置 JSON",
    )
    parser.add_argument("--draw-roi", action="store_true", help="在画面上画出各槽位矩形（调试用）")
    parser.add_argument(
        "--decision-state",
        type=str,
        default="../config/decision_state.json",
        help="数学识别写入的决策状态JSON",
    )
    parser.add_argument(
        "--nav-target",
        type=str,
        default="../config/nav_target.json",
        help="自动导航目标输出（供 slot_nav_dispatcher 读取）",
    )
    return parser.parse_args()


def load_decision_state(path: Path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None


def write_nav_target(path: Path, slot_id: str, target_class: str, mod4, candidates):
    payload = {
        "slot_id": str(slot_id),
        "target_class": target_class,
        "mod4": mod4,
        "candidates": [str(s) for s in candidates],
        "updated_at": time.time(),
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def normalize_class_name(name: str) -> str:
    n = str(name).strip().lower()
    mapping = {
        "tool": "工具",
        "instrument": "仪器",
        "food": "食品",
        "medicine": "药品",
        "工具": "工具",
        "仪器": "仪器",
        "食品": "食品",
        "药品": "药品",
    }
    return mapping.get(n, str(name).strip())


def test_camera(camera_id):
    """测试摄像头是否可用"""
    cap = cv2.VideoCapture(camera_id)
    if cap.isOpened():
        ret, _ = cap.read()
        cap.release()
        return ret
    return False


def find_available_cameras():
    """查找可用的摄像头"""
    available = []
    for i in range(5):  # 检查 0-4
        if test_camera(i):
            available.append(i)
    return available


def coerce_source(src: str):
    """将字符串转为合适的输入源"""
    if src.isdigit():
        camera_id = int(src)
        available_cameras = find_available_cameras()
        
        print(f"可用摄像头: {available_cameras}")
        
        if camera_id in available_cameras:
            print(f"使用摄像头 {camera_id}")
            return camera_id
        else:
            print(f"摄像头 {camera_id} 不可用")
            if available_cameras:
                fallback = available_cameras[0]
                print(f"自动切换到摄像头 {fallback}")
                return fallback
            else:
                print("未找到可用摄像头")
                sys.exit(1)
    return src


def main():
    args = parse_args()
    
    # 检查权重文件（允许相对路径，基于 _PROJ_DIR 解析）
    w = Path(args.weights)
    if not w.is_absolute():
        w = _PROJ_DIR / w
    if not w.exists():
        print(f"找不到权重: {w}")
        print(f"请放在 {_PROJ_DIR / 'weights/'} 目录或使用 --weights 指定绝对路径")
        sys.exit(1)

    print(f"加载模型: {w}")
    model = YOLO(str(w))
    source = coerce_source(args.source)

    roi_slots = None
    roi_frame_wh = None
    slot_label_map = {}
    roi_size_warned = False
    if args.roi_config:
        roi_path = Path(args.roi_config)
        if not roi_path.is_absolute():
            roi_path = _PROJ_DIR / roi_path
        if not roi_path.exists():
            print(f"找不到 ROI 配置: {roi_path}")
            sys.exit(1)
        roi_slots, roi_frame_wh = load_slots_roi_config(roi_path)
        slot_label_map = {s.slot_id: s.label for s in roi_slots}
        print(f"已加载槽位 ROI: {roi_path}，共 {len(roi_slots)} 个区域")

    decision_state_path = Path(args.decision_state)
    if not decision_state_path.is_absolute():
        decision_state_path = _PROJ_DIR / decision_state_path
    print(f"决策状态文件: {decision_state_path}")

    nav_target_path = Path(args.nav_target)
    if not nav_target_path.is_absolute():
        nav_target_path = _PROJ_DIR / nav_target_path
    print(f"自动导航目标文件: {nav_target_path}")
    
    # 如果是摄像头输入，添加实时显示的额外设置
    if isinstance(source, int):
        print("摄像头模式启动:")
        print("- 按 'q' 键退出")
        print("- 按 's' 键保存当前帧")
        print("- 检测结果实时显示")

    try:
        # 如果是摄像头输入，使用自定义显示逻辑
        if isinstance(source, int):
            # 直接使用 OpenCV 显示摄像头
            cap = cv2.VideoCapture(source)
            if not cap.isOpened():
                print(f"无法打开摄像头 {source}")
                return
                
            # 设置摄像头属性
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.cam_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.cam_height)
            cap.set(cv2.CAP_PROP_FPS, 30)

            window_name = "YOLO Detection"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, args.window_width, args.window_height)
            
            print("摄像头已启动，按 'q' 退出, 按 's' 保存截图")
            
            frame_count = 0
            last_detections = []  # 保存上一帧的检测结果
            detection_stable_count = {}  # 记录每个检测框的稳定帧数
            last_slot_report = None  # 记录上一次终端输出，避免重复刷屏
            last_decision_print = None  # 记录上次决策输出，避免重复刷屏
            
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("无法读取摄像头帧")
                    break

                if roi_slots and roi_frame_wh and not roi_size_warned:
                    warn_frame_size_mismatch(frame, roi_frame_wh)
                    roi_size_warned = True
                
                frame_count += 1
                current_detections = []
                
                # 每3帧进行一次检测（降低CPU占用）
                if frame_count % 3 == 0:
                    # 使用模型进行预测
                    results = model.predict(
                        source=frame,
                        imgsz=args.img,
                        conf=args.conf,
                        device=args.device,
                        save=False,
                        show=False,
                        verbose=False
                    )
                    
                    # 收集当前帧检测结果
                    if results and len(results) > 0:
                        result = results[0]
                        if hasattr(result, 'boxes') and result.boxes is not None:
                            boxes = result.boxes
                            for box in boxes:
                                if hasattr(box, 'xyxy'):
                                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                                    conf = float(box.conf[0]) if hasattr(box, 'conf') else 0
                                    cls_id = int(box.cls[0]) if hasattr(box, 'cls') else 0
                                    
                                    detection = {
                                        'box': (x1, y1, x2, y2),
                                        'conf': conf,
                                        'cls_id': cls_id,
                                        'center': ((x1+x2)//2, (y1+y2)//2)
                                    }
                                    current_detections.append(detection)
                    
                    # 更新稳定性计数
                    new_stable_count = {}
                    for det in current_detections:
                        det_key = f"{det['cls_id']}_{det['center'][0]//50}_{det['center'][1]//50}"  # 区域化匹配
                        
                        # 检查是否与上一帧检测结果相似
                        is_stable = False
                        for last_det in last_detections:
                            last_key = f"{last_det['cls_id']}_{last_det['center'][0]//50}_{last_det['center'][1]//50}"
                            if det_key == last_key:
                                is_stable = True
                                break
                        
                        if is_stable:
                            new_stable_count[det_key] = detection_stable_count.get(det_key, 0) + 1
                        else:
                            new_stable_count[det_key] = 1
                    
                    detection_stable_count = new_stable_count
                    last_detections = current_detections.copy()
                else:
                    # 非检测帧，使用上一帧结果
                    current_detections = last_detections

                if args.draw_roi and roi_slots:
                    draw_slot_rois(frame, roi_slots)
                
                # 稳定的检测列表（用于槽位编号，避免抖动框抢 slot）
                stable_dets: list = []
                for det in current_detections:
                    det_key = f"{det['cls_id']}_{det['center'][0]//50}_{det['center'][1]//50}"
                    stable_count = detection_stable_count.get(det_key, 0)
                    if stable_count >= 2:
                        stable_dets.append(det)

                if roi_slots and stable_dets:
                    stable_dets = assign_slots_for_detections(stable_dets, roi_slots)

                stable_by_key = {
                    f"{d['cls_id']}_{d['center'][0]//50}_{d['center'][1]//50}": d
                    for d in stable_dets
                }

                # 终端输出：编号 -> 物资箱类别（仅在结果变化时打印）
                slot_report = {}
                if roi_slots:
                    for d in stable_dets:
                        slot_id = d.get("slot_id")
                        if slot_id is None:
                            continue
                        cls_id = d.get("cls_id", -1)
                        cls_name = model.names.get(cls_id, f"class_{cls_id}") if hasattr(model, 'names') else f"class_{cls_id}"
                        slot_text = slot_label_map.get(slot_id, str(slot_id + 1))
                        slot_report[str(slot_text)] = cls_name

                if slot_report != last_slot_report:
                    if slot_report:
                        def _slot_sort_key(item):
                            k = item[0]
                            return (0, int(k)) if str(k).isdigit() else (1, str(k))

                        print("\n[ROI编号识别结果]")
                        for slot_text, cls_name in sorted(slot_report.items(), key=_slot_sort_key):
                            print(f"  编号 {slot_text} -> {cls_name}")
                    else:
                        print("\n[ROI编号识别结果] 当前没有命中ROI的稳定目标")
                    last_slot_report = dict(slot_report)

                # 读取数学链路决策，并输出“应该抓哪个编号”
                decision = load_decision_state(decision_state_path)
                if decision and slot_report:
                    target_class = normalize_class_name(decision.get("target_class", ""))
                    target_slots = []
                    for slot_text, cls_name in slot_report.items():
                        if normalize_class_name(cls_name) == target_class:
                            target_slots.append(slot_text)
                    target_slots.sort(key=lambda x: (0, int(x)) if str(x).isdigit() else (1, str(x)))
                    decision_signature = (decision.get("mod4"), target_class, tuple(target_slots))
                    if decision_signature != last_decision_print:
                        mod4 = decision.get("mod4")
                        if target_slots:
                            joined = ", ".join(target_slots)
                            print(f"[抓取决策] mod4={mod4} -> 目标={target_class} -> 去抓编号: {joined}")
                            write_nav_target(
                                nav_target_path,
                                slot_id=target_slots[0],
                                target_class=target_class,
                                mod4=mod4,
                                candidates=target_slots,
                            )
                        else:
                            print(f"[抓取决策] mod4={mod4} -> 目标={target_class}，当前ROI中未找到对应物资箱")
                        last_decision_print = decision_signature

                # 绘制稳定的检测结果（至少连续出现3次）
                for i, det in enumerate(current_detections):
                    det_key = f"{det['cls_id']}_{det['center'][0]//50}_{det['center'][1]//50}"
                    stable_count = detection_stable_count.get(det_key, 0)
                    
                    # 只显示稳定的检测结果
                    if stable_count >= 2:  # 至少连续2次检测
                        x1, y1, x2, y2 = det['box']
                        conf = det['conf']
                        cls_id = det['cls_id']

                        slot_id = None
                        if det_key in stable_by_key:
                            slot_id = stable_by_key[det_key].get("slot_id")
                        
                        # 获取类别名称并转为英文
                        cls_name = model.names.get(cls_id, f"class_{cls_id}") if hasattr(model, 'names') else f"class_{cls_id}"
                        
                        # 中文到英文的映射
                        class_mapping = {
                            '工具': 'Tool',
                            '仪器': 'Instrument', 
                            '食品': 'Food',
                            '药品': 'Medicine'
                        }
                        english_name = class_mapping.get(cls_name, cls_name)
                        
                        # 根据稳定性调整颜色深度
                        alpha = min(1.0, stable_count / 5.0)  # 越稳定颜色越深
                        color_intensity = int(255 * alpha)
                        
                        # 绘制边界框 (绿色，稳定性越高越亮)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, color_intensity, 0), 3)
                        
                        # 创建标签背景 (黑色半透明)
                        if slot_id is not None:
                            slot_text = slot_label_map.get(slot_id, str(slot_id + 1))
                            label = f"No.{slot_text} {english_name}: {conf:.2f}"
                        else:
                            label = f"{english_name}: {conf:.2f}"
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                        cv2.rectangle(frame, (x1, y1-label_size[1]-10), (x1+label_size[0], y1), (0, 0, 0), -1)
                        
                        # 绘制白色文字标签
                        cv2.putText(frame, label, (x1, y1-5), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

                        # 在框右侧额外显示编号，便于快速查看槽位号
                        if slot_id is not None:
                            slot_text = slot_label_map.get(slot_id, str(slot_id + 1))
                            side_text = f"#{slot_text}"
                            side_x = min(x2 + 6, frame.shape[1] - 60)
                            side_y = max(y1 + 22, 20)
                            cv2.putText(
                                frame,
                                side_text,
                                (side_x, side_y),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.65,
                                (255, 0, 255),
                                2,
                            )
                
                # 添加帧率和提示信息
                cv2.putText(frame, f"Frame: {frame_count}", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                cv2.putText(frame, "Press 'q' to quit, 's' to save", (10, 60), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # 显示图像
                cv2.imshow(window_name, frame)
                
                # 检查按键
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("用户按 'q' 退出")
                    break
                elif key == ord('s'):
                    save_path = f"screenshot_{frame_count}.jpg"
                    cv2.imwrite(save_path, frame)
                    print(f"截图保存到: {save_path}")
            
            cap.release()
            cv2.destroyAllWindows()
            
        else:
            # 非摄像头输入使用原来的方式
            results = model.predict(
                source=source,
                imgsz=args.img,
                conf=args.conf,
                device=args.device,
                project=args.project,
                name=args.name,
                save=args.save_video,
                show=args.show,
                verbose=True
            )
            
            # 非摄像头输入的处理
            if not isinstance(source, int):
                results_list = list(results) if hasattr(results, '__iter__') else [results]
                if results_list:
                    first = results_list[0]
                    print("推理完成，结果样例：", first)
                    save_dir = getattr(first, 'save_dir', None)
                    if save_dir is not None:
                        print("可视化结果保存于：", Path(save_dir).resolve())
                    
    except KeyboardInterrupt:
        print("\n检测被用户中断")
    except Exception as e:
        print(f"检测过程出错: {e}")
    finally:
        cv2.destroyAllWindows()
        print("检测结束")


if __name__ == "__main__":
    main()