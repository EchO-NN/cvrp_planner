#!/usr/bin/env python3
"""Export a native occupancy map from the current Isaac warehouse scene."""

from __future__ import annotations

import argparse
import json
import traceback
from pathlib import Path


def _default_stage_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    candidates = [
        repo_root / "scripts" / "isaac" / "generated" / "full_warehouse_nav_robots.example.usd",
        repo_root / "scripts" / "isaac" / "generated" / "full_warehouse_robots.example.usd",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def _default_output_image_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "cvrp_planner" / "maps" / "full_warehouse_native_map.pgm"


def _default_output_yaml_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "cvrp_planner" / "maps" / "full_warehouse_native_map.yaml"


def _default_output_params_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "scripts" / "isaac" / "generated" / "full_warehouse_native_map.params.yaml"


def _default_output_meta_path() -> Path:
    repo_root = Path(__file__).resolve().parents[2]
    return repo_root / "scripts" / "isaac" / "generated" / "full_warehouse_native_map.meta.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a ROS occupancy map from an Isaac scene.")
    parser.add_argument("--stage", default=str(_default_stage_path()), help="USD stage to export from.")
    parser.add_argument("--output-image", default=str(_default_output_image_path()), help="Output occupancy image (.pgm).")
    parser.add_argument("--output-yaml", default=str(_default_output_yaml_path()), help="Output ROS map YAML.")
    parser.add_argument(
        "--output-params",
        default=str(_default_output_params_path()),
        help="Output params template for cvrp_planner.",
    )
    parser.add_argument("--output-meta", default=str(_default_output_meta_path()), help="Output metadata JSON.")
    parser.add_argument("--cell-size", type=float, default=0.05, help="Occupancy map cell size in stage units.")
    parser.add_argument("--headless", action="store_true", help="Run Isaac Sim headless.")
    parser.add_argument(
        "--exclude-robots",
        action="store_true",
        help="Deactivate /Robots before generating the map.",
    )
    return parser.parse_args()


def _log(message: str) -> None:
    print(message, flush=True)


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _enable_extensions(extension_names) -> None:
    try:
        from isaacsim.core.utils.extensions import enable_extension
    except ImportError:
        from omni.isaac.core.utils.extensions import enable_extension

    for extension_name in extension_names:
        enable_extension(extension_name)
        _log(f"[omap] 已启用扩展: {extension_name}")


def _measure_scene_bounds(stage, exclude_prefixes):
    from pxr import Gf, Usd, UsdGeom

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        includedPurposes=[
            UsdGeom.Tokens.default_,
            UsdGeom.Tokens.render,
            UsdGeom.Tokens.proxy,
        ],
        useExtentsHint=True,
    )

    total_bounds = Gf.BBox3d()
    counted = 0

    def _is_excluded(path_text: str) -> bool:
        for prefix in exclude_prefixes:
            normalized = prefix.rstrip("/")
            if path_text == normalized or path_text.startswith(normalized + "/"):
                return True
        return False

    for prim in stage.Traverse():
        if not prim.IsActive():
            continue
        path_text = prim.GetPath().pathString
        if _is_excluded(path_text):
            continue
        if not prim.IsA(UsdGeom.Boundable):
            continue
        try:
            aligned_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            total_bounds = Gf.BBox3d.Combine(total_bounds, Gf.BBox3d(aligned_range))
            counted += 1
        except Exception:
            continue

    if counted == 0:
        raise RuntimeError("没有量到任何可用于导图的几何体。")

    bounds_range = total_bounds.GetRange()
    min_point = bounds_range.GetMin()
    max_point = bounds_range.GetMax()
    return {
        "min_x": float(min_point[0]),
        "min_y": float(min_point[1]),
        "min_z": float(min_point[2]),
        "max_x": float(max_point[0]),
        "max_y": float(max_point[1]),
        "max_z": float(max_point[2]),
        "width_x": float(max_point[0] - min_point[0]),
        "depth_y": float(max_point[1] - min_point[1]),
        "height_z": float(max_point[2] - min_point[2]),
        "center_x": float((min_point[0] + max_point[0]) * 0.5),
        "center_y": float((min_point[1] + max_point[1]) * 0.5),
        "center_z": float((min_point[2] + max_point[2]) * 0.5),
    }


def _write_p2_pgm(path: Path, rows, maxval: int = 255) -> None:
    _ensure_parent(path)
    height = len(rows)
    width = len(rows[0]) if rows else 0
    with path.open("w", encoding="ascii") as file_obj:
        file_obj.write("P2\n")
        file_obj.write("# generated by export_scene_occupancy_map.py\n")
        file_obj.write(f"{width} {height}\n")
        file_obj.write(f"{maxval}\n")
        for row in rows:
            file_obj.write(" ".join(str(int(value)) for value in row))
            file_obj.write("\n")


def _update_location(omap, start_location, lower_bound, upper_bound) -> None:
    omap.set_transform(
        (start_location[0], start_location[1], start_location[2]),
        (lower_bound[0], lower_bound[1], lower_bound[2]),
        (upper_bound[0], upper_bound[1], upper_bound[2]),
    )
    omap.update()


def _stage_point_to_grid_index(point, min_bound, max_bound, cell_size, width, height):
    x = float(point[0])
    y = float(point[1])

    col = int((x - float(min_bound[0])) / cell_size)
    row = int((float(max_bound[1]) - y) / cell_size)

    if row < 0:
        row = 0
    elif row >= height:
        row = height - 1

    if col < 0:
        col = 0
    elif col >= width:
        col = width - 1

    return row, col


def _build_ros_pgm_rows(omap, width, height, cell_size):
    min_bound = omap.get_min_bound()
    max_bound = omap.get_max_bound()
    rows = [[205 for _ in range(width)] for _ in range(height)]

    free_positions = list(omap.get_free_positions())
    occupied_positions = list(omap.get_occupied_positions())

    for point in free_positions:
        row, col = _stage_point_to_grid_index(point, min_bound, max_bound, cell_size, width, height)
        rows[row][col] = 254

    # Occupied cells should override free cells if the generator reports both.
    for point in occupied_positions:
        row, col = _stage_point_to_grid_index(point, min_bound, max_bound, cell_size, width, height)
        rows[row][col] = 0

    return rows, free_positions, occupied_positions


def _write_ros_yaml(path: Path, image_path: Path, resolution_m: float, origin_xy) -> None:
    _ensure_parent(path)
    image_ref = image_path.name if image_path.parent == path.parent else str(image_path)
    contents = (
        f"image: {image_ref}\n"
        f"resolution: {resolution_m:.9f}\n"
        f"origin: [{origin_xy[0]:.9f}, {origin_xy[1]:.9f}, 0.0000]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
    )
    path.write_text(contents, encoding="utf-8")


def _write_planner_params(path: Path, ros_yaml_path: Path) -> None:
    _ensure_parent(path)
    contents = (
        "params_generator:\n"
        "  ros__parameters:\n"
        f"    map_path: \"{ros_yaml_path.resolve()}\"\n"
        "cvrp_planner_node:\n"
        "  ros__parameters:\n"
        f"    map_path: \"{ros_yaml_path.resolve()}\"\n"
    )
    path.write_text(contents, encoding="utf-8")


def main() -> int:
    args = _parse_args()
    stage_path = Path(args.stage).expanduser().resolve()
    output_image_path = Path(args.output_image).expanduser().resolve()
    output_yaml_path = Path(args.output_yaml).expanduser().resolve()
    output_params_path = Path(args.output_params).expanduser().resolve()
    output_meta_path = Path(args.output_meta).expanduser().resolve()

    try:
        from isaacsim import SimulationApp
    except ImportError:
        try:
            from omni.isaac.kit import SimulationApp
        except ImportError as exc:
            raise RuntimeError("请使用 Isaac Sim 自带的 python.sh 运行此脚本。") from exc

    _log(f"[omap] 启动导图脚本: stage={stage_path}")
    app = SimulationApp({"headless": bool(args.headless)})

    try:
        import omni.timeline
        import omni.usd

        _enable_extensions(["isaacsim.asset.gen.omap"])
        for _ in range(10):
            app.update()

        from isaacsim.asset.gen.omap.bindings import _omap
        from isaacsim.core.utils.stage import get_stage_units

        context = omni.usd.get_context()
        if not context.open_stage(str(stage_path)):
            raise RuntimeError(f"无法打开场景: {stage_path}")
        for _ in range(20):
            app.update()

        stage = context.get_stage()
        if args.exclude_robots:
            robots_prim = stage.GetPrimAtPath("/Robots")
            if robots_prim.IsValid():
                robots_prim.SetActive(False)
                _log("[omap] 已临时停用 /Robots，避免机器人被画进占据图")
                for _ in range(10):
                    app.update()

        bounds = _measure_scene_bounds(stage, exclude_prefixes=["/Robots"] if args.exclude_robots else [])
        _log(
            "[omap] 场景边界: "
            f"x=[{bounds['min_x']:.3f}, {bounds['max_x']:.3f}] "
            f"y=[{bounds['min_y']:.3f}, {bounds['max_y']:.3f}] "
            f"z=[{bounds['min_z']:.3f}, {bounds['max_z']:.3f}]"
        )

        origin = [bounds["center_x"], bounds["center_y"], 0.0]
        lower_bound = [
            bounds["min_x"] - origin[0],
            bounds["min_y"] - origin[1],
            bounds["min_z"] - origin[2],
        ]
        upper_bound = [
            bounds["max_x"] - origin[0],
            bounds["max_y"] - origin[1],
            bounds["max_z"] - origin[2],
        ]

        omap = _omap.acquire_omap_interface()
        try:
            _update_location(omap, origin, lower_bound, upper_bound)
            omap.set_cell_size(args.cell_size)
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            for _ in range(5):
                app.update()
            omap.generate()
            for _ in range(5):
                app.update()
            timeline.stop()

            dims = omap.get_dimensions()
            if dims.x == 0 or dims.y == 0:
                raise RuntimeError("Occupancy map 为空，请确认场景里有 collision geometry。")

            width_px = int(dims.x)
            height_px = int(dims.y)
            output_rows, free_positions, occupied_positions = _build_ros_pgm_rows(
                omap,
                width_px,
                height_px,
                args.cell_size,
            )
            if not occupied_positions:
                raise RuntimeError(
                    "导出的 occupancy map 没有任何 occupied cells。"
                    "这通常说明 collision geometry 没被 omap 识别，或者场景/导图参数不对。",
                )
            _write_p2_pgm(output_image_path, output_rows)

            stage_units = get_stage_units()
            meters_per_unit = float(stage_units)
            resolution_m = float(args.cell_size * meters_per_unit)
            min_bound = omap.get_min_bound()
            max_bound = omap.get_max_bound()
            ros_origin_xy = [
                float(min_bound[0]) * meters_per_unit,
                float(min_bound[1]) * meters_per_unit,
            ]
            _write_ros_yaml(output_yaml_path, output_image_path, resolution_m, ros_origin_xy)
            _write_planner_params(output_params_path, output_yaml_path)

            unique_values = {}
            for row in output_rows:
                for value in row:
                    unique_values[value] = unique_values.get(value, 0) + 1

            metadata = {
                "stage": str(stage_path),
                "output_image": str(output_image_path),
                "output_yaml": str(output_yaml_path),
                "output_params": str(output_params_path),
                "cell_size_stage_units": float(args.cell_size),
                "resolution_m": resolution_m,
                "rotation": 0,
                "ros_origin": ros_origin_xy,
                "dimensions_px": {"width": width_px, "height": height_px},
                "scene_bounds": bounds,
                "omap_bounds": {
                    "min_x": float(min_bound[0]),
                    "min_y": float(min_bound[1]),
                    "max_x": float(max_bound[0]),
                    "max_y": float(max_bound[1]),
                },
                "cell_counts": {
                    "free": len(free_positions),
                    "occupied": len(occupied_positions),
                    "pgm_values": unique_values,
                },
                "notes": [
                    "Image uses standard ROS occupancy map orientation.",
                    "Use the generated ROS YAML as map_path in cvrp_planner for direct loading.",
                ],
            }
            _ensure_parent(output_meta_path)
            output_meta_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

            _log(f"[omap] 占据图已保存: {output_image_path}")
            _log(f"[omap] ROS YAML 已保存: {output_yaml_path}")
            _log(f"[omap] 规划参数模板已保存: {output_params_path}")
            _log(
                f"[omap] omap 统计: occupied={len(occupied_positions)}, "
                f"free={len(free_positions)}, pgm_values={unique_values}"
            )
            _log(
                f"[omap] 输出尺寸: {width_px} x {height_px}, "
                f"resolution={resolution_m:.3f} m/pixel, origin={ros_origin_xy}"
            )
        finally:
            _omap.release_omap_interface(omap)
    except Exception:
        _log("[omap] 导图失败，完整异常如下:")
        _log(traceback.format_exc())
        raise
    finally:
        app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
