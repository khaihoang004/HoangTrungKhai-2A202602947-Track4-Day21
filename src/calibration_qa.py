"""Reproducible LiDAR-camera calibration QA on the supplied KITTI and nuScenes subsets.

Run from the repository root: python3 -m src.calibration_qa --help
All labels, images and point clouds are read from the unmodified supplied datasets.
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from starter.datasets import list_frames, load_frame
from starter.projection import (overlay_points, perturb_extrinsic,
                                project_velo_to_image, velo_to_cam)


DATASETS = {
    "kitti": "data/kitti_mini",
    "nuscenes": "data/nuscenes_mini_subset",
}
SWEEPS = [("yaw_deg", x) for x in (0.0, 0.5, 1.0, 2.0, 3.0)] + [
    ("tx_m", x) for x in (0.02, 0.05, 0.10)
]


def object_indices(points_cam: np.ndarray, obj) -> np.ndarray:
    """Indices of LiDAR returns within the original camera-frame 3D GT box."""
    h, w, length = obj.dimensions
    c, s = np.cos(obj.rotation_y), np.sin(obj.rotation_y)
    rot = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    local = (points_cam - obj.location) @ rot
    return np.flatnonzero(
        np.isfinite(local).all(axis=1)
        & (np.abs(local[:, 0]) <= length / 2)
        & (local[:, 1] >= -h) & (local[:, 1] <= 0)
        & (np.abs(local[:, 2]) <= w / 2)
    )


def within_box(uv: np.ndarray, bbox: np.ndarray) -> np.ndarray:
    x1, y1, x2, y2 = bbox
    return (np.isfinite(uv).all(axis=1) & (uv[:, 0] >= x1) & (uv[:, 0] <= x2)
            & (uv[:, 1] >= y1) & (uv[:, 1] <= y2))


def projected_map(points: np.ndarray, calib, image_shape) -> tuple[np.ndarray, np.ndarray, int]:
    uv, _, mask = project_velo_to_image(points, calib, image_shape)
    full_uv = np.full((len(points), 2), np.nan)
    full_uv[mask] = uv
    return full_uv, mask, int(mask.sum())


def write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)


def make_overlay(fr: dict, out: Path, title: str, calib=None) -> None:
    calib = calib or fr["calib"]
    uv, depth, _ = project_velo_to_image(fr["points"], calib, fr["image"].shape)
    vis = overlay_points(fr["image"], uv, depth, radius=1)
    for obj in fr["labels"]:
        x1, y1, x2, y2 = np.rint(obj.bbox).astype(int)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.rectangle(vis, (0, 0), (min(vis.shape[1], 820), 34), (0, 0, 0), -1)
    cv2.putText(vis, title, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(out), vis):
        raise OSError(f"Could not write {out}")


def make_failure(fr: dict, obj, indices: np.ndarray, out: Path) -> None:
    """Show the same 3D-box LiDAR returns before and after a 2-degree yaw error."""
    if len(indices) == 0:
        return
    baseline, _, _ = projected_map(fr["points"], fr["calib"], fr["image"].shape)
    shifted, _, _ = projected_map(
        fr["points"], perturb_extrinsic(fr["calib"], yaw_deg=2), fr["image"].shape)
    x1, y1, x2, y2 = obj.bbox
    pad = 90
    h, w = fr["image"].shape[:2]
    left, top = max(0, int(x1) - pad), max(0, int(y1) - pad)
    right, bottom = min(w, int(x2) + pad), min(h, int(y2) + pad)
    panels = []
    for label, uv, color in (("0 deg", baseline, (0, 255, 0)),
                             ("2 deg yaw", shifted, (0, 0, 255))):
        panel = fr["image"][top:bottom, left:right].copy()
        box = np.rint([x1-left, y1-top, x2-left, y2-top]).astype(int)
        cv2.rectangle(panel, tuple(box[:2]), tuple(box[2:]), (255, 255, 0), 2)
        for u, v in uv[indices]:
            if np.isfinite(u) and left <= u < right and top <= v < bottom:
                cv2.circle(panel, (int(u-left), int(v-top)), 3, color, -1)
        hit = np.count_nonzero(within_box(uv[indices], obj.bbox))
        cv2.rectangle(panel, (0, 0), (min(panel.shape[1], 270), 30), (0, 0, 0), -1)
        cv2.putText(panel, f"{label}: {hit}/{len(indices)} in 2D box", (5, 21),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1)
        panels.append(panel)
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), np.hstack(panels))


def run(out_root: Path, selected: list[str]) -> None:
    details: list[dict] = []
    object_rows: list[dict] = []
    failure_candidates = []
    demo_frames = {"000019": "near", "000011": "middle", "000004": "far"}

    for dataset in selected:
        root = DATASETS[dataset]
        frames = list_frames(root)
        print(f"{dataset}: {len(frames)} frames", flush=True)
        for frame_id in frames:
            fr = load_frame(root, frame_id)
            pts = fr["points"]
            cam = velo_to_cam(pts[:, :3], fr["calib"])
            uv0, mask0, _ = projected_map(pts, fr["calib"], fr["image"].shape)
            finite_xyz = np.isfinite(pts[:, :3]).all(axis=1)
            obj_info = []
            for number, obj in enumerate(fr["labels"]):
                ids = object_indices(cam, obj)
                ids = ids[mask0[ids] & (cam[ids, 2] >= 5) & (cam[ids, 2] <= 60)]
                if len(ids) < 5:
                    continue
                obj_info.append((number, obj, ids))

            if dataset == "kitti" and frame_id in demo_frames:
                make_overlay(fr, out_root / "figures" / f"demo_{demo_frames[frame_id]}_{frame_id}.png",
                             f"KITTI {frame_id} | baseline projection")
            if dataset == "nuscenes" and frame_id == "scene-0103_010":
                make_overlay(fr, out_root / "figures" / "demo_nuscenes_scene-0103_010.png",
                             "nuScenes scene-0103_010 | ego motion compensated")

            for sweep, level in SWEEPS:
                calib = (fr["calib"] if level == 0 else
                         perturb_extrinsic(fr["calib"], yaw_deg=level) if sweep == "yaw_deg" else
                         perturb_extrinsic(fr["calib"], t_xyz_m=(level, 0, 0)))
                uv, mask, inside = projected_map(pts, calib, fr["image"].shape)
                common = mask0 & mask & (cam[:, 2] >= 5) & (cam[:, 2] <= 60)
                shift = np.linalg.norm(uv[common] - uv0[common], axis=1)
                hits = 0
                total = 0
                for number, obj, ids in obj_info:
                    base_hit = np.count_nonzero(within_box(uv0[ids], obj.bbox))
                    hit = np.count_nonzero(within_box(uv[ids], obj.bbox))
                    total += len(ids)
                    hits += hit
                    object_rows.append(dict(dataset=dataset, frame=frame_id, object_id=number,
                                            class_name=obj.type, depth_m=round(float(np.median(cam[ids, 2])), 3),
                                            sweep=sweep, level=level, object_points=len(ids),
                                            box_hits=hit, box_retention_pct=round(100 * hit / len(ids), 3)))
                    if (dataset == "kitti" and sweep == "yaw_deg" and level == 2
                            and obj.type == "Pedestrian" and len(ids) >= 50
                            and 10 <= np.median(cam[ids, 2]) <= 35):
                        loss = (base_hit - hit) / len(ids)
                        failure_candidates.append((loss, frame_id, number))
                details.append(dict(dataset=dataset, frame=frame_id, sweep=sweep, level=level,
                                    valid_points=int(finite_xyz.sum()), fov_points=inside,
                                    object_points=total, box_hits=hits,
                                    median_shift_px=round(float(np.median(shift)), 3) if len(shift) else ""))

    groups = {}
    for row in details:
        key = (row["dataset"], row["sweep"], row["level"])
        groups.setdefault(key, []).append(row)
    summary = []
    for (dataset, sweep, level), rows in groups.items():
        valid = sum(r["valid_points"] for r in rows)
        obj_count = sum(r["object_points"] for r in rows)
        summary.append(dict(dataset=dataset, sweep=sweep, level=level, frames=len(rows),
                            valid_points=valid, fov_points=sum(r["fov_points"] for r in rows),
                            fov_pct=round(100 * sum(r["fov_points"] for r in rows) / valid, 3),
                            object_points=obj_count, box_hits=sum(r["box_hits"] for r in rows),
                            box_retention_pct=round(100 * sum(r["box_hits"] for r in rows) / obj_count, 3)
                            if obj_count else "",
                            median_frame_shift_px=round(float(np.median([
                                r["median_shift_px"] for r in rows if r["median_shift_px"] != ""])), 3)))

    write_csv(out_root / "calibration_sweep.csv", summary,
              ["dataset", "sweep", "level", "frames", "valid_points", "fov_points", "fov_pct",
               "object_points", "box_hits", "box_retention_pct", "median_frame_shift_px"])
    write_csv(out_root / "calibration_by_frame.csv", details,
              ["dataset", "frame", "sweep", "level", "valid_points", "fov_points", "object_points",
               "box_hits", "median_shift_px"])
    write_csv(out_root / "calibration_by_object.csv", object_rows,
              ["dataset", "frame", "object_id", "class_name", "depth_m", "sweep", "level",
               "object_points", "box_hits", "box_retention_pct"])

    for sweep, xlabel, filename in (("yaw_deg", "Yaw drift (degrees)", "yaw_sweep.png"),
                                    ("tx_m", "LiDAR x translation (m)", "translation_sweep.png")):
        fig, axes = plt.subplots(1, 2, figsize=(10, 4), constrained_layout=True)
        for dataset in selected:
            rows = [r for r in summary if r["dataset"] == dataset and r["sweep"] == sweep]
            line, = axes[0].plot([r["level"] for r in rows], [r["box_retention_pct"] for r in rows],
                                 "o-", label=dataset)
            axes[1].plot([r["level"] for r in rows], [r["fov_pct"] for r in rows],
                         "o-", label=dataset)
            if sweep == "yaw_deg":
                axes[0].axhline(rows[0]["box_retention_pct"] - 10,
                                color=line.get_color(), linestyle="--", alpha=0.55,
                                label=f"{dataset}: baseline - 10 pp")
        axes[0].set(title="3D GT returns landing in 2D box", xlabel=xlabel,
                    ylabel="Box hit rate (%)")
        axes[1].set(title="All finite LiDAR returns inside image", xlabel=xlabel,
                    ylabel="Inside FOV (%)")
        for ax in axes:
            ax.grid(alpha=0.3)
            ax.legend()
        plot = out_root / "figures" / filename
        plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(plot, dpi=160)
        plt.close(fig)

    if "kitti" in selected and failure_candidates:
        _, frame_id, number = max(failure_candidates)
        fr = load_frame(DATASETS["kitti"], frame_id)
        cam = velo_to_cam(fr["points"][:, :3], fr["calib"])
        _, mask0, _ = projected_map(fr["points"], fr["calib"], fr["image"].shape)
        obj = fr["labels"][number]
        ids = object_indices(cam, obj)
        ids = ids[mask0[ids] & (cam[ids, 2] >= 5) & (cam[ids, 2] <= 60)]
        fail_name = f"fail_01_yaw_2deg_{frame_id}_obj{number}.png"
        make_failure(fr, obj, ids, out_root / "figures" / fail_name)
        print(f"failure: frame={frame_id}, object={number}, loss={max(failure_candidates)[0]:.1%}")

    for row in summary:
        print(row)


def main() -> None:
    parser = argparse.ArgumentParser(description="Sweep LiDAR-camera yaw and translation drift over both real datasets")
    parser.add_argument("--datasets", nargs="+", choices=tuple(DATASETS), default=list(DATASETS),
                        help="datasets to evaluate (default: both)")
    parser.add_argument("--out-dir", type=Path, default=Path("results"), help="output directory")
    args = parser.parse_args()
    run(args.out_dir, args.datasets)


if __name__ == "__main__":
    main()
