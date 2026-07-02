"""
固定机位下：使用预先标定的矩形 ROI，把检测框中心映射到 slot_id（0~7）。
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional

import cv2
import numpy as np


@dataclass(frozen=True)
class SlotDef:
    slot_id: int
    label: str
    rect: tuple[int, int, int, int]  # x1, y1, x2, y2


def _point_in_rect(cx: int, cy: int, rect: tuple[int, int, int, int]) -> bool:
    x1, y1, x2, y2 = rect
    return x1 <= cx <= x2 and y1 <= cy <= y2


def load_slots_roi_config(path: str | Path) -> tuple[list[SlotDef], tuple[int, int] | None]:
    p = Path(path)
    raw = json.loads(p.read_text(encoding="utf-8"))
    frame_wh: tuple[int, int] | None = None
    if "frame_wh" in raw and isinstance(raw["frame_wh"], list) and len(raw["frame_wh"]) == 2:
        frame_wh = (int(raw["frame_wh"][0]), int(raw["frame_wh"][1]))

    slots: list[SlotDef] = []
    for item in raw.get("slots", []):
        rect = item["rect"]
        slots.append(
            SlotDef(
                slot_id=int(item["id"]),
                label=str(item.get("label", item["id"])),
                rect=(int(rect[0]), int(rect[1]), int(rect[2]), int(rect[3])),
            )
        )

    slots.sort(key=lambda s: s.slot_id)
    return slots, frame_wh


def assign_slot_id(cx: int, cy: int, slots: Iterable[SlotDef]) -> Optional[int]:
    for s in slots:
        if _point_in_rect(cx, cy, s.rect):
            return s.slot_id
    return None


def assign_slots_for_detections(
    detections: list[dict[str, Any]],
    slots: Iterable[SlotDef],
) -> list[dict[str, Any]]:
    """
    为每条检测补充 slot_id（可能为 None）。
    同一个 slot 内有多个框时，仅保留置信度最高者，其他框 slot_id 置 None。
    """
    slot_list = list(slots)
    out: list[dict[str, Any]] = []

    for det in detections:
        d = dict(det)
        cx, cy = d["center"]
        d["slot_id"] = assign_slot_id(int(cx), int(cy), slot_list)
        out.append(d)

    best_by_slot: dict[int, int] = {}
    for i, d in enumerate(out):
        sid = d.get("slot_id")
        if sid is None:
            continue
        conf = float(d.get("conf", 0.0))
        if sid not in best_by_slot:
            best_by_slot[sid] = i
            continue
        j = best_by_slot[sid]
        if conf > float(out[j].get("conf", 0.0)):
            best_by_slot[sid] = i

    winners = set(best_by_slot.values())
    for i, d in enumerate(out):
        sid = d.get("slot_id")
        if sid is not None and i not in winners:
            d["slot_id"] = None

    return out


def draw_slot_rois(
    frame: np.ndarray,
    slots: Iterable[SlotDef],
    color: tuple[int, int, int] = (128, 128, 255),
    thickness: int = 2,
) -> None:
    for s in slots:
        x1, y1, x2, y2 = s.rect
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
        cv2.putText(
            frame,
            str(s.label),
            (x1 + 4, y1 + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
        )


def warn_frame_size_mismatch(frame: np.ndarray, frame_wh: tuple[int, int] | None) -> None:
    if frame_wh is None:
        return
    h, w = frame.shape[:2]
    if (w, h) != frame_wh:
        print(
            f"[slot_roi] 警告: 当前帧尺寸 {w}x{h} 与配置 frame_wh {frame_wh[0]}x{frame_wh[1]} 不一致，"
            "请重新标定 ROI 或更新 frame_wh。"
        )