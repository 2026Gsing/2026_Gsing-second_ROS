"""
摄像头 + YOLO 模型快速测试脚本

功能：
  - 交互式选择：先选摄像头 → 再选模型
  - 加载任意 YOLO 模型（task3.pt / math12.pt / 自定义）
  - USB 摄像头实时推理
  - 显示检测框 + 标签 + 置信度 + FPS
  - 支持 ROI 槽位叠加显示

用法：
  # 交互模式（不传参，自动弹菜单选摄像头 + 模型）
  python src/camera_test.py

  # 直接指定（跳过交互）
  python src/camera_test.py --weights weights/task3.pt --source 1

  # 开启 ROI 槽位叠加
  python src/camera_test.py --draw-roi

  # 保存测试视频
  python src/camera_test.py --save-video test_result.avi
"""

import argparse
import csv
import os
import sys
import time
from contextlib import contextmanager
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

_SRC_DIR = Path(__file__).resolve().parent          # vision/src/
_PROJ_DIR = _SRC_DIR.parent                         # vision/
if str(_SRC_DIR) not in sys.path:
    sys.path.insert(0, str(_SRC_DIR))

# ── 模型类别名称 ─────────────────────────────────────────────────
MODEL_CLASSES: dict[str, list[str]] = {
    "task3": [
        "工具", "仪器", "食品", "药品",
    ],
    "math12": [
        "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
        "+", "-", "x", "÷", "(", ")",
    ],
}


def _guess_model_key(weights_path: Path) -> str | None:
    """根据文件名猜测模型类别表"""
    name = weights_path.stem.lower()
    for key in MODEL_CLASSES:
        if key in name:
            return key
    return None


# ═══════════════════════════════════════════════════════════════
#  交互式选择：摄像头 + 模型
# ═══════════════════════════════════════════════════════════════

@contextmanager
def _silent_cv():
    """临时抑制 OpenCV 底层 C 警告（探测摄像头时用）。

    不 redirect stderr（会破坏 GStreamer 管道初始化）。
    改为通过 OpenCV 的 logging 机制和临时环境变量抑制。
    """
    # 优先用 OpenCV 自带的 logging 级别控制 (4.8+)
    try:
        prev = cv2.utils.logging.getLogLevel()
        cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_ERROR)
    except AttributeError:
        prev = None

    # OPENCV_LOG_LEVEL 环境变量（4.6 及以后均受支持）
    old_env = os.environ.get("OPENCV_LOG_LEVEL")
    os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

    try:
        yield
    finally:
        if prev is not None:
            try:
                cv2.utils.logging.setLogLevel(prev)
            except AttributeError:
                pass
        if old_env is not None:
            os.environ["OPENCV_LOG_LEVEL"] = old_env
        else:
            os.environ.pop("OPENCV_LOG_LEVEL", None)


def find_available_cameras(max_id: int = 10) -> list[tuple[int, tuple[int, int]]]:
    """
    探测可用摄像头，返回列表 [(cam_id, (max_w, max_h)), ...]。
    在单次打开中完成：读帧验证 + 最大分辨率探测。
    避免反复开关导致 UVC 驱动卡死。
    """
    # 只尝试系统上存在的 /dev/video* 设备，避免 V4L2 警告
    existing_video = set()
    for d in range(max_id):
        if os.path.exists(f"/dev/video{d}"):
            existing_video.add(d)

    # 候选分辨率（从高到低）
    candidates = [
        (3840, 2160), (2592, 1944), (2560, 1440), (1920, 1080),
        (1600, 1200), (1440, 1080), (1366, 768), (1280, 1024),
        (1280, 960), (1280, 720), (1024, 768), (800, 600), (640, 480),
    ]

    result: list[tuple[int, tuple[int, int]]] = []
    for i in existing_video:
        # 针对非捕获设备（metadata）的 V4L2 警告，临时静音
        devnull = os.open(os.devnull, os.O_WRONLY)
        old_stderr = os.dup(2)
        os.dup2(devnull, 2)
        os.close(devnull)
        cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
        os.dup2(old_stderr, 2)
        os.close(old_stderr)

        if not cap.isOpened():
            continue

        # 连续读 2 帧验证是真摄像头
        with _silent_cv():
            ok1, _ = cap.read()
            ok2, _ = cap.read()

        if not (ok1 and ok2):
            cap.release()
            continue

        # 在此次打开中顺便探测最大分辨率
        with _silent_cv():
            default_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            default_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            best_w, best_h = default_w, default_h
            for cw, ch in candidates:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, cw)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ch)
                aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                if aw >= best_w and ah >= best_h:
                    best_w, best_h = aw, ah

        cap.release()
        result.append((i, (best_w, best_h)))
    return result


def interactive_select_camera() -> tuple[int, tuple[int, int]]:
    """交互式选择摄像头，返回 (cam_id, (max_w, max_h))"""
    avail = find_available_cameras(10)

    print("=" * 50)
    print("  📷 摄像头选择")
    print("=" * 50)

    if not avail:
        print("  ⚠️  未检测到任何可用摄像头！")
        print("  请检查摄像头连接后重试。")
        raise SystemExit(1)

    print(f"\n  发现 {len(avail)} 个可用摄像头:\n")
    for i, (cam_id, (w, h)) in enumerate(avail):
        print(f"    [{i + 1}] 摄像头 #{cam_id}  —  最大分辨率 {w}×{h}")

    print()
    while True:
        try:
            choice = input(f"  请选择摄像头 (1-{len(avail)}，回车默认 1): ").strip()
            if not choice:
                idx = 0
            else:
                idx = int(choice) - 1
            if 0 <= idx < len(avail):
                cam_id, (w, h) = avail[idx]
                print(f"  ✅ 已选: 摄像头 #{cam_id}  ({w}×{h})\n")
                return cam_id, (w, h)
            else:
                print(f"  ❌ 请输入 1-{len(avail)} 之间的数字")
        except ValueError:
            print("  ❌ 请输入有效数字")


def get_model_info(pt_path: Path) -> str:
    """获取模型简介：文件名 + 类别数 + 已知类别名"""
    size_mb = pt_path.stat().st_size / (1024 * 1024)
    key = _guess_model_key(pt_path)
    if key and key in MODEL_CLASSES:
        cls_list = MODEL_CLASSES[key]
        return f"{pt_path.name}  ({len(cls_list)} 类: {' · '.join(cls_list)})  [{size_mb:.0f} MB]"
    else:
        return f"{pt_path.name}  (未知类别)  [{size_mb:.0f} MB]"


def interactive_select_model() -> Path:
    """交互式选择模型权重"""
    weights_dir = _PROJ_DIR / "weights"
    pt_files = sorted(weights_dir.glob("*.pt"))

    print("=" * 50)
    print("  🧠 模型选择")
    print("=" * 50)

    if not pt_files:
        print(f"  ⚠️  {weights_dir} 下没有 .pt 文件！")
        print("  请将 YOLO 权重放入该目录。")
        raise SystemExit(1)

    print(f"\n  发现 {len(pt_files)} 个模型:\n")
    for i, pt in enumerate(pt_files):
        info = get_model_info(pt)
        print(f"    [{i + 1}] {info}")

    # 默认选中序号（优先 task3，否则第 1 个）
    default_idx = 0
    for j, pt in enumerate(pt_files):
        if "task3" in pt.stem.lower():
            default_idx = j
            break

    print()
    while True:
        try:
            prompt = f"  请选择模型 (1-{len(pt_files)}，回车默认 {default_idx + 1}): "
            choice = input(prompt).strip()
            if not choice:
                idx = default_idx
            else:
                idx = int(choice) - 1
            if 0 <= idx < len(pt_files):
                selected = pt_files[idx]
                print(f"  ✅ 已选: {selected.name}\n")
                return selected
            else:
                print(f"  ❌ 请输入 1-{len(pt_files)} 之间的数字")
        except ValueError:
            print("  ❌ 请输入有效数字")


def resolve_source(src_str: str) -> tuple[int | str, tuple[int, int] | None]:
    """
    '0'/'1' → int(摄像头编号)；否则原样返回（文件路径）。
    返回 (source, max_resolution_or_None)。
    """
    if not src_str.isdigit():
        return src_str, None

    cam_id = int(src_str)
    with _silent_cv():
        cap = cv2.VideoCapture(cam_id, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"  摄像头 {cam_id} 无法打开", end="")
            avail = find_available_cameras()
            if avail:
                fallback, (fw, fh) = avail[0]
                print(f"，自动切换到 #{fallback} ({fw}×{fh})")
                return fallback, (fw, fh)
            print("，未找到任何摄像头")
            raise SystemExit(1)
        ok1, _ = cap.read()
        ok2, _ = cap.read()
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        cap.release()

    if not (ok1 and ok2):
        print(f"  摄像头 {cam_id} 无法读取画面")
        avail = find_available_cameras()
        if avail:
            fallback, (fw, fh) = avail[0]
            print(f"，自动切换到 #{fallback} ({fw}×{fh})")
            return fallback, (fw, fh)
        print("，未找到可用摄像头")
        raise SystemExit(1)

    return cam_id, (w, h)


# ═══════════════════════════════════════════════════════════════
#  绘制函数
# ═══════════════════════════════════════════════════════════════

def draw_debug_overlay(
    frame: np.ndarray,
    fps: float,
    inference_ms: float,
    det_count: int,
) -> None:
    """左上角信息叠加层"""
    lines = [
        f"FPS: {fps:.1f}",
        f"Infer: {inference_ms:.0f} ms",
        f"Detections: {det_count}",
    ]
    y0 = 24
    for i, text in enumerate(lines):
        y = y0 + i * 24
        cv2.putText(
            frame, text, (12, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
        )


def draw_detection(
    frame: np.ndarray,
    x1: int, y1: int, x2: int, y2: int,
    label: str,
    conf: float,
    color: tuple[int, int, int] = (0, 255, 0),
) -> None:
    """绘制单个检测框 + 标签"""
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    text = f"{label} {conf:.2f}"
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 0, 0), -1)
    cv2.putText(
        frame, text, (x1 + 3, y1 - 4),
        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
    )


# ═══════════════════════════════════════════════════════════════
#  主流程
# ═══════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="YOLO 摄像头测试工具")
    parser.add_argument("--weights", type=str, default=None,
                        help="模型权重路径（不传则交互选择）")
    parser.add_argument("--source", type=str, default=None,
                        help="摄像头编号 (0/1) 或文件路径（不传则交互选择）")
    parser.add_argument("--conf", type=float, default=0.35, help="置信度阈值")
    parser.add_argument("--imgsz", type=int, default=640, help="推理输入尺寸")
    parser.add_argument("--device", type=str, default="0", help="推理设备 (0/cpu)")
    parser.add_argument("--cam-width", type=int, default=None, help="摄像头采集宽度")
    parser.add_argument("--cam-height", type=int, default=None, help="摄像头采集高度")
    parser.add_argument("--show-fps", action="store_true", default=True,
                        help="显示 FPS / 推理耗时")
    parser.add_argument("--draw-roi", action="store_true",
                        help="叠加显示 ROI 槽位 (需 --roi-config)")
    parser.add_argument("--roi-config", type=str, default=None,
                        help="ROI 槽位配置文件路径")
    parser.add_argument("--save-video", type=str, default=None,
                        help="保存视频到文件 (如 test.avi)")
    parser.add_argument("--save-csv", type=str, default=None,
                        help="保存检测日志到 CSV 文件")
    parser.add_argument("--show", action="store_true", default=True,
                        help="显示预览窗口")
    parser.add_argument("--no-show", action="store_false", dest="show",
                        help="不显示预览窗口（纯后台测试）")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    # ── 交互选择：摄像头 ─────────────────────────────────────
    # 优先级：CLI 指定分辨率 > 探测分辨率 > 默认 1280×720
    has_cli_res = args.cam_width is not None and args.cam_height is not None
    if args.source is not None:
        source, probe_res = resolve_source(args.source)
        cam_resolution = (
            (args.cam_width, args.cam_height) if has_cli_res else
            probe_res if probe_res is not None else
            (1280, 720)
        )
    else:
        source, cam_resolution = interactive_select_camera()
    is_camera = isinstance(source, int)

    # ── 交互选择：模型 ───────────────────────────────────────
    if args.weights is not None:
        w = Path(args.weights)
        if not w.is_absolute():
            w = _PROJ_DIR / w
    else:
        w = interactive_select_model()

    if not w.exists():
        print(f"[错误] 找不到权重: {w.resolve()}")
        print(f"       请放在 {_PROJ_DIR / 'weights/'} 下，或使用 --weights 指定路径")
        raise SystemExit(1)

    # ── 加载模型 ──────────────────────────────────────────────
    model_key = _guess_model_key(w)
    class_names = MODEL_CLASSES.get(model_key, []) if model_key else []
    if class_names:
        print(f"  模型类别 ({model_key}): {class_names}")
    else:
        print(f"  模型类别: 使用模型内置 names")

    print(f"  加载模型: {w.resolve()}")
    t0 = time.perf_counter()
    model = YOLO(str(w))
    print(f"  加载耗时: {time.perf_counter() - t0:.1f}s")

    # 优先用模型自带的 names
    if hasattr(model, "names") and model.names:
        known_names = model.names
    else:
        known_names = {i: n for i, n in enumerate(class_names)}

    # ── 加载 ROI ──────────────────────────────────────────────
    roi_slots = None
    roi_draw_fn = None
    if args.draw_roi:
        roi_path = args.roi_config or str(_PROJ_DIR / "config" / "slots_roi.json")
        rp = Path(roi_path)
        if not rp.is_absolute():
            rp = _PROJ_DIR / rp
        if rp.exists():
            try:
                from slot_roi import draw_slot_rois, load_slots_roi_config
                roi_slots, _ = load_slots_roi_config(rp)
                roi_draw_fn = draw_slot_rois
                print(f"  加载 ROI: {rp} ({len(roi_slots)} 个槽位)")
            except Exception as e:
                print(f"  [警告] ROI 加载失败: {e}")
        else:
            print(f"  [警告] ROI 文件不存在: {rp}")

    # ── 打开摄像头 ─────────────────────────────────────────────
    if is_camera:
        res_w, res_h = cam_resolution
        cap = cv2.VideoCapture(source, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"[错误] 无法打开摄像头 {source}")
            raise SystemExit(1)
        # 部分 UVC 驱动在频繁开关后需要短暂延迟
        time.sleep(0.3)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, res_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, res_h)
        cap.set(cv2.CAP_PROP_FPS, 30)
        actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if actual_w == 0 or actual_h == 0:
            actual_w, actual_h = res_w, res_h
        # 试读一帧验证
        ok = False
        for _ in range(3):
            ok, _ = cap.read()
            if ok:
                break
            time.sleep(0.1)
        status = "成功" if ok else "失败"
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        print(f"  摄像头 {source}: {actual_w}×{actual_h} @ {actual_fps:.0f} fps (试读{status})")
    else:
        cap = None
        print(f"  输入源: {source}")

    # ── 视频保存 ──────────────────────────────────────────────
    video_writer = None
    if args.save_video and is_camera:
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        video_writer = cv2.VideoWriter(
            args.save_video, fourcc, 20.0,
            (actual_w, actual_h),
        )
        print(f"  视频保存到: {args.save_video}")

    # ── CSV 日志 ──────────────────────────────────────────────
    csv_fp = None
    csv_writer = None
    if args.save_csv:
        csv_fp = open(args.save_csv, "w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_fp)
        csv_writer.writerow(["frame", "cls_id", "cls_name", "conf",
                             "x1", "y1", "x2", "y2", "infer_ms"])
        print(f"  CSV 日志: {args.save_csv}")

    # ── 主循环 ─────────────────────────────────────────────────
    print("\n  🎯 按 'q' 退出  |  按 's' 保存截图  |  按 'f' 显示/隐藏 FPS\n")

    window_name = f"Camera #{source} - {w.name}"
    frame_count = 0
    show_fps = args.show_fps

    fps_alpha = 0.1
    smooth_infer = 0.0
    expected_fps = 0.0

    try:
        while True:
            if is_camera:
                ret, frame = cap.read()
                if not ret:
                    print("[结束] 摄像头读取失败")
                    break
            else:
                break  # 非摄像头模式在循环外处理

            frame_count += 1

            # ── 推理 ─────────────────────────────────────────
            t_infer = time.perf_counter()
            results = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                verbose=False,
                save=False,
                show=False,
            )
            infer_ms = (time.perf_counter() - t_infer) * 1000
            smooth_infer = infer_ms if frame_count == 1 else (
                smooth_infer * (1 - fps_alpha) + infer_ms * fps_alpha
            )
            expected_fps = 1000 / max(smooth_infer, 1)

            # ── 绘制检测框 ───────────────────────────────────────
            det_count = 0
            if results and len(results) > 0:
                result = results[0]
                if hasattr(result, "boxes") and result.boxes is not None:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        cls_name = known_names.get(cls_id, f"cls_{cls_id}")
                        draw_detection(frame, x1, y1, x2, y2, cls_name, conf)
                        det_count += 1

                        if csv_writer:
                            csv_writer.writerow([
                                frame_count, cls_id, cls_name, round(conf, 4),
                                x1, y1, x2, y2, round(infer_ms, 1),
                            ])

            # ROI 叠加
            if roi_slots and roi_draw_fn:
                roi_draw_fn(frame, roi_slots)

            # FPS / 信息
            if show_fps:
                draw_debug_overlay(frame, expected_fps, smooth_infer, det_count)

            # 底部快捷键提示
            cv2.putText(
                frame, "[q]退出  [s]截图  [f]FPS开关",
                (12, frame.shape[0] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180, 180, 180), 1,
            )

            # ── 显示 / 保存 ─────────────────────────────────
            if video_writer:
                video_writer.write(frame)

            if args.show:
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    print("  用户按 q 退出")
                    break
                elif key == ord("s"):
                    snap = f"screenshot_{frame_count}.jpg"
                    cv2.imwrite(snap, frame)
                    print(f"  截图保存: {snap}")
                elif key == ord("f"):
                    show_fps = not show_fps

        # ── 非摄像头输入 ─────────────────────────────────────
        if not is_camera:
            print(f"  运行推理: {source}")
            results = model.predict(
                source=source,
                imgsz=args.imgsz,
                conf=args.conf,
                device=args.device,
                save=True,
                show=args.show,
                verbose=True,
            )
            if results:
                r = results[0]
                if hasattr(r, "save_dir") and r.save_dir:
                    print(f"  结果保存: {Path(r.save_dir).resolve()}")

    except KeyboardInterrupt:
        print("\n  用户中断")
    except Exception as e:
        print(f"\n  [错误] {e}")
        import traceback
        traceback.print_exc()
    finally:
        if is_camera and cap:
            cap.release()
        if video_writer:
            video_writer.release()
        if csv_fp:
            csv_fp.close()
        cv2.destroyAllWindows()

        if frame_count > 0 and is_camera:
            print(f"\n  ── 测试统计 ──")
            print(f"  总帧数:     {frame_count}")
            print(f"  平均推理:   {smooth_infer:.0f} ms")
            print(f"  预计帧率:   {expected_fps:.1f} FPS")


def quick_test() -> None:
    """一键快速测试：交互选择摄像头 + 模型"""
    import sys

    weights_dir = _PROJ_DIR / "weights"
    pt_files = sorted(weights_dir.glob("*.pt"))
    if not pt_files:
        print(f"[错误] {weights_dir} 下没有 .pt 权重文件")
        return

    # 进入交互式选择
    cam_id, (cam_w, cam_h) = interactive_select_camera()
    model_path = interactive_select_model()

    # 构造参数并调用 main
    sys.argv = [
        sys.argv[0],
        "--source", str(cam_id),
        "--cam-width", str(cam_w),
        "--cam-height", str(cam_h),
        "--weights", str(model_path),
        "--show",
    ]
    main()


if __name__ == "__main__":
    # 如果没传任何参数 → 进入交互模式
    if len(sys.argv) == 1:
        quick_test()
    else:
        main()
