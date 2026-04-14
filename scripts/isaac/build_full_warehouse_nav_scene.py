#!/usr/bin/env python3
"""Build a ROS-ready multi-robot warehouse scene for Isaac Sim.

This script is intentionally opinionated for the current repository:

1. It keeps the official ``full_warehouse.usd`` environment.
2. It spawns multiple ``Nova_Carter_ROS`` robots with independent namespaces.
3. It patches the embedded ROS 2 OmniGraphs so each robot gets namespaced
   ``cmd_vel / odom / scan / tf`` topics.
4. It writes a JSON manifest and a ROS parameter YAML template that match the
   current ``cvrp_planner`` package.

Use Isaac Sim's ``python.sh`` to run it.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List


ROS_READY_ROBOT_ASSET = "/Isaac/Samples/ROS2/Robots/Nova_Carter_ROS.usd"


def _log(message: str) -> None:
    print(message, flush=True)


def _ensure_parent(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def _json_string(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False)


def _load_config(config_path: Path) -> Dict[str, Any]:
    with config_path.open("r", encoding="utf-8") as file_obj:
        return json.load(file_obj)


def _resolve_output_path(config_path: Path, explicit_path: str | None) -> Path:
    if explicit_path:
        return Path(explicit_path).expanduser().resolve()
    generated_dir = config_path.resolve().parent / "generated"
    return (generated_dir / f"{config_path.stem}.usd").resolve()


def _resolve_source_stage(config: Dict[str, Any], assets_root_path: str | None) -> str:
    stage_path = config.get("stage_path")
    if stage_path:
        resolved = Path(stage_path).expanduser().resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"找不到 stage_path 指定的场景: {resolved}")
        return str(resolved)

    stage_url = config.get("stage_url", "/Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
    if stage_url.startswith("/"):
        if not assets_root_path:
            raise RuntimeError("无法解析官方 Isaac 资产路径，请确认 get_assets_root_path() 可用。")
        return assets_root_path + stage_url

    resolved = Path(stage_url).expanduser().resolve()
    if not resolved.is_file():
        raise FileNotFoundError(f"找不到 stage_url 指定的本地场景: {resolved}")
    return str(resolved)


def _validate_config(config: Dict[str, Any]) -> None:
    robots = config.get("robots", [])
    if not robots:
        raise RuntimeError("配置里至少需要 1 台机器人。")

    min_separation = float(config.get("minimum_separation_m", 2.5))
    seen_names: set[str] = set()
    seen_namespaces: set[str] = set()
    seen_paths: set[str] = set()

    for robot in robots:
        name = robot["name"]
        namespace = robot.get("namespace", name).strip("/")
        position = robot.get("position", [])
        prim_path = robot.get("prim_path", f"/Robots/{name}")

        if robot.get("type", "nova_carter_ros") != "nova_carter_ros":
            raise RuntimeError(
                "build_full_warehouse_nav_scene.py 目前只支持 type=nova_carter_ros。"
            )
        if name in seen_names:
            raise RuntimeError(f"机器人名称重复: {name}")
        if namespace in seen_namespaces:
            raise RuntimeError(f"机器人 namespace 重复: {namespace}")
        if prim_path in seen_paths:
            raise RuntimeError(f"机器人 prim_path 重复: {prim_path}")
        if len(position) != 3:
            raise RuntimeError(f"机器人 {name} 的 position 必须是 3 个数字: {position}")

        seen_names.add(name)
        seen_namespaces.add(namespace)
        seen_paths.add(prim_path)

    for index, robot_a in enumerate(robots):
        pos_a = robot_a["position"]
        for robot_b in robots[index + 1 :]:
            pos_b = robot_b["position"]
            dx = float(pos_a[0]) - float(pos_b[0])
            dy = float(pos_a[1]) - float(pos_b[1])
            distance_xy = (dx * dx + dy * dy) ** 0.5
            if distance_xy < 1e-6:
                raise RuntimeError(
                    f"机器人 {robot_a['name']} 和 {robot_b['name']} 的初始位置完全重合: {pos_a}"
                )
            if distance_xy < min_separation:
                raise RuntimeError(
                    f"机器人 {robot_a['name']} 和 {robot_b['name']} 的初始间距过小: "
                    f"{distance_xy:.2f} m < {min_separation:.2f} m"
                )


def _fully_qualified_topic(namespace: str, topic_name: str) -> str:
    topic_name = topic_name.strip()
    if not topic_name:
        return ""
    if topic_name.startswith("/"):
        return topic_name
    namespace = namespace.strip("/")
    if not namespace:
        return f"/{topic_name}"
    return f"/{namespace}/{topic_name}"


def _normalize_topic_name(topic_name: str) -> str:
    topic_name = topic_name.strip()
    return topic_name.lstrip("/")


def _topic_matches(topic: str, suffix: str) -> bool:
    return topic == suffix or topic.endswith(suffix)


def _first_topic(topics: Iterable[str], suffixes: Iterable[str]) -> str:
    for suffix in suffixes:
        for topic in topics:
            if _topic_matches(topic, suffix):
                return topic
    return ""


def _all_topics(topics: Iterable[str], suffixes: Iterable[str]) -> List[str]:
    matches: List[str] = []
    for topic in topics:
        if any(_topic_matches(topic, suffix) for suffix in suffixes):
            matches.append(topic)
    return sorted(set(matches))


def _render_ros_params_yaml(manifest: Dict[str, Any]) -> str:
    cvrp_export = manifest["cvrp_export"]
    common_block = f"""    robot_names: '{_json_string(manifest["robot_names"])}'
    robot_pose_topics: '{_json_string(manifest["robot_pose_topics"])}'
    robot_pose_type: "{manifest["robot_pose_type"]}"
    robot_starts: '{_json_string(manifest["robot_starts"])}'
    odometry_is_relative: {"true" if manifest["odometry_is_relative"] else "false"}
    odometry_origin_positions: '{_json_string(manifest["odometry_origin_positions"])}'
    odometry_origin_yaws_deg: '{_json_string(manifest["odometry_origin_yaws_deg"])}'
    customers: '{_json_string(manifest["customers"])}'
    demands: '{_json_string(manifest["demands"])}'
    num_vehicles: {manifest["num_vehicles"]}
    vehicle_capacities: '{_json_string(manifest["vehicle_capacities"])}'
    follow_waypoints_actions: '{_json_string(manifest["follow_waypoints_actions"])}'
    auto_dispatch: {"true" if cvrp_export["auto_dispatch"] else "false"}
    map_path: ""
    calculating_time: {cvrp_export["calculating_time"]}
    inflate_size: {cvrp_export["inflate_size"]}
    real_length: {cvrp_export["real_length"]}
    origin_pixel: '{_json_string(cvrp_export["origin_pixel"])}'
    origin_world: '{_json_string(cvrp_export["origin_world"])}'
    frame_id: "{cvrp_export["frame_id"]}"
    pose_wait_timeout: {cvrp_export["pose_wait_timeout"]}
    nav2_wait_timeout: {cvrp_export["nav2_wait_timeout"]}
    visualize: {"true" if cvrp_export["visualize"] else "false"}"""
    return f"""params_generator:
  ros__parameters:
{common_block}
cvrp_planner_node:
  ros__parameters:
{common_block}
"""


def _write_manifest(manifest_path: Path, manifest: Dict[str, Any]) -> None:
    _ensure_parent(manifest_path)
    with manifest_path.open("w", encoding="utf-8") as file_obj:
        json.dump(manifest, file_obj, ensure_ascii=False, indent=2)
        file_obj.write("\n")


def _write_params_yaml(params_path: Path, manifest: Dict[str, Any]) -> None:
    _ensure_parent(params_path)
    with params_path.open("w", encoding="utf-8") as file_obj:
        file_obj.write(_render_ros_params_yaml(manifest))


def _set_ogn_attr(og: Any, attribute_path: str, value: Any) -> None:
    try:
        attr = og.Controller.attribute(attribute_path)
        if attr.is_valid():
            og.Controller.set(attr, value)
    except Exception:
        return


def _configure_ros_ready_robot(
    stage: Any,
    og: Any,
    asset_prim_path: str,
    namespace: str,
    ros_bridge_config: Dict[str, Any],
) -> List[str]:
    from pxr import Usd

    root_prim = stage.GetPrimAtPath(asset_prim_path)
    if not root_prim.IsValid():
        raise RuntimeError(f"找不到机器人 prim: {asset_prim_path}")

    collected_topics: List[str] = []
    for prim in Usd.PrimRange(root_prim):
        for attr in prim.GetAttributes():
            attr_name = attr.GetName()
            if attr_name == "inputs:nodeNamespace":
                attr.Set(namespace)
            elif attr_name == "inputs:topicName":
                topic_name = attr.Get()
                if isinstance(topic_name, str) and topic_name.strip():
                    normalized = _normalize_topic_name(topic_name)
                    attr.Set(normalized)
                    collected_topics.append(_fully_qualified_topic(namespace, normalized))

    if ros_bridge_config.get("disable_cameras", True):
        for camera_graph in ("front_hawk", "left_hawk", "right_hawk", "back_hawk"):
            for node_name in ("left_camera_render_product", "right_camera_render_product"):
                _set_ogn_attr(
                    og,
                    f"{asset_prim_path}/{camera_graph}/{node_name}.inputs:enabled",
                    False,
                )

    _set_ogn_attr(
        og,
        f"{asset_prim_path}/ros_lidars/front_2d_lidar_render_product.inputs:enabled",
        bool(ros_bridge_config.get("enable_front_2d_lidar", True)),
    )
    _set_ogn_attr(
        og,
        f"{asset_prim_path}/ros_lidars/back_2d_lidar_render_product.inputs:enabled",
        bool(ros_bridge_config.get("enable_back_2d_lidar", False)),
    )
    _set_ogn_attr(
        og,
        f"{asset_prim_path}/ros_lidars/front_3d_lidar_render_product.inputs:enabled",
        bool(ros_bridge_config.get("enable_front_3d_lidar", False)),
    )

    return sorted(set(topic for topic in collected_topics if topic))


def _add_clock_graph(og: Any) -> str:
    keys = og.Controller.Keys
    graph_path = "/ROS_Clock"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
                ("PublishClock.inputs:topicName", "clock"),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    return "/clock"


def _build_manifest(
    config: Dict[str, Any],
    output_stage: Path,
    source_stage: str,
    robots: List[Dict[str, Any]],
    clock_topic: str,
) -> Dict[str, Any]:
    cvrp_export = config.get("cvrp_export", {})
    frame_id = cvrp_export.get("frame_id") or config.get("frame_id") or "map"

    robot_names = [robot["name"] for robot in robots]
    namespaces = [robot["namespace"] for robot in robots]
    starts = [[float(robot["position"][0]), float(robot["position"][1])] for robot in robots]
    start_yaws_deg = [float(robot.get("yaw_deg", 0.0)) for robot in robots]
    capacities = [
        int(robot.get("capacity", config.get("vehicle_capacity_default", 20)))
        for robot in robots
    ]

    nav_topics = {
        "clock": clock_topic,
        "cmd_vel": [robot.get("cmd_vel_topic", "") for robot in robots],
        "odom": [robot.get("odom_topic", "") for robot in robots],
        "scan": [
            robot["scan_topics"][0] if robot.get("scan_topics") else ""
            for robot in robots
        ],
        "tf": [robot.get("tf_topic", "") for robot in robots],
        "tf_static": [robot.get("tf_static_topic", "") for robot in robots],
    }

    manifest = {
        "scene_name": config.get("scene_name", output_stage.stem),
        "scene_usd": str(output_stage),
        "source_stage": source_stage,
        "frame_id": frame_id,
        "robot_names": robot_names,
        "robot_namespaces": namespaces,
        "robot_starts": starts,
        "odometry_is_relative": True,
        "odometry_origin_positions": starts,
        "odometry_origin_yaws_deg": start_yaws_deg,
        "robot_pose_topics": [
            robot.get("odom_topic") or f"/{robot['namespace']}/chassis/odom"
            for robot in robots
        ],
        "robot_pose_type": "odometry",
        "follow_waypoints_actions": [
            f"/{namespace}/follow_waypoints" for namespace in namespaces
        ],
        "nav2_expected_topics": nav_topics,
        "num_vehicles": len(robot_names),
        "vehicle_capacities": capacities,
        "customers": config.get("customers", []),
        "demands": config.get("demands", []),
        "cvrp_export": {
            "real_length": float(cvrp_export.get("real_length", 100.0)),
            "origin_pixel": cvrp_export.get("origin_pixel", [0, 0]),
            "origin_world": cvrp_export.get("origin_world", [-50.0, 20.0]),
            "frame_id": frame_id,
            "calculating_time": int(cvrp_export.get("calculating_time", 3)),
            "inflate_size": int(cvrp_export.get("inflate_size", 5)),
            "pose_wait_timeout": float(cvrp_export.get("pose_wait_timeout", 10.0)),
            "nav2_wait_timeout": float(cvrp_export.get("nav2_wait_timeout", 15.0)),
            "auto_dispatch": bool(cvrp_export.get("auto_dispatch", True)),
            "visualize": bool(cvrp_export.get("visualize", False)),
        },
        "robots": robots,
    }
    return manifest


def _build_scene(
    app: Any,
    config: Dict[str, Any],
    source_stage: str,
    output_stage: Path,
    assets_root_path: str | None,
    headless: bool,
) -> Dict[str, Any]:
    import carb
    import omni.graph.core as og
    import omni.kit.app
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, Sdf, UsdGeom

    context = omni.usd.get_context()
    _log(f"[nav_scene] 打开源场景: {source_stage}")
    if not context.open_stage(source_stage):
        raise RuntimeError(f"无法打开源场景: {source_stage}")

    for _ in range(10):
        app.update()

    stage = context.get_stage()
    UsdGeom.Xform.Define(stage, "/Robots")

    viewport = config.get("viewport", {})
    eye = viewport.get("eye")
    target = viewport.get("target")
    if eye and target and not headless:
        set_camera_view(
            eye=[float(value) for value in eye],
            target=[float(value) for value in target],
            camera_prim_path="/OmniverseKit_Persp",
        )

    ros_bridge_config = config.get("ros_bridge", {})
    publish_without_verification = bool(
        ros_bridge_config.get("publish_without_verification", True)
    )
    carb.settings.get_settings().set_bool(
        "/exts/isaacsim.ros2.bridge/publish_without_verification",
        publish_without_verification,
    )

    manifest_robots: List[Dict[str, Any]] = []
    robot_asset_path = assets_root_path + ROS_READY_ROBOT_ASSET if assets_root_path else ROS_READY_ROBOT_ASSET

    for robot in config.get("robots", []):
        name = robot["name"]
        namespace = robot.get("namespace", name).strip("/")
        prim_path = robot.get("prim_path", f"/Robots/{name}")
        asset_prim_path = robot.get("asset_prim_path", f"{prim_path}/Asset")
        position = [float(value) for value in robot["position"]]
        yaw_deg = float(robot.get("yaw_deg", 0.0))

        if stage.GetPrimAtPath(prim_path).IsValid():
            raise RuntimeError(f"机器人 prim 已存在，请换个 prim_path: {prim_path}")

        spawn_prim = UsdGeom.Xform.Define(stage, prim_path).GetPrim()
        xform_api = UsdGeom.XformCommonAPI(spawn_prim)
        xform_api.SetTranslate(Gf.Vec3d(*position))
        xform_api.SetRotate(
            Gf.Vec3f(0.0, 0.0, yaw_deg),
            UsdGeom.XformCommonAPI.RotationOrderXYZ,
        )

        _log(f"[nav_scene] 添加机器人 {name}: {robot_asset_path} -> {asset_prim_path}")
        add_reference_to_stage(robot_asset_path, asset_prim_path)
        spawn_prim.CreateAttribute("robotNamespace", Sdf.ValueTypeNames.String).Set(namespace)
        spawn_prim.CreateAttribute("robotType", Sdf.ValueTypeNames.String).Set("nova_carter_ros")

        for _ in range(5):
            app.update()

        topics = _configure_ros_ready_robot(stage, og, asset_prim_path, namespace, ros_bridge_config)
        robot_entry = {
            "name": name,
            "namespace": namespace,
            "requested_type": robot["type"],
            "type": "nova_carter_ros",
            "prim_path": prim_path,
            "asset_prim_path": asset_prim_path,
            "position": position,
            "yaw_deg": yaw_deg,
            "asset_path": robot_asset_path,
            "topics": topics,
            "cmd_vel_topic": _first_topic(topics, ["/cmd_vel"]),
            "odom_topic": _first_topic(topics, ["/odom", "/chassis/odom"]),
            "scan_topics": _all_topics(topics, ["/scan", "/laser_scan"]),
            "tf_topic": _first_topic(topics, ["/tf"]),
            "tf_static_topic": _first_topic(topics, ["/tf_static"]),
            "capacity": int(robot.get("capacity", config.get("vehicle_capacity_default", 20))),
        }
        manifest_robots.append(robot_entry)

    for _ in range(10):
        omni.kit.app.get_app().update()

    clock_topic = _add_clock_graph(og)
    for _ in range(5):
        omni.kit.app.get_app().update()

    _ensure_parent(output_stage)
    _log(f"[nav_scene] 开始保存场景: {output_stage}")
    if not context.save_as_stage(str(output_stage)):
        raise RuntimeError(f"无法保存输出场景: {output_stage}")

    for _ in range(5):
        omni.kit.app.get_app().update()

    _log(f"[nav_scene] 输出场景已保存: {output_stage}")
    return _build_manifest(config, output_stage, source_stage, manifest_robots, clock_topic)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a ROS-ready multi-robot warehouse scene with Nova Carter ROS assets.",
    )
    parser.add_argument(
        "--config",
        default=str(Path(__file__).resolve().parent / "full_warehouse_nav_robots.example.json"),
        help="Robot placement JSON config for the ROS-ready navigation scene.",
    )
    parser.add_argument(
        "--save-path",
        default=None,
        help="Output USD path. Defaults to scripts/isaac/generated/<config>.usd",
    )
    parser.add_argument(
        "--assets-root",
        default=None,
        help="Override Isaac assets root path. Example: omniverse://localhost/NVIDIA/Assets/Isaac/5.1",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Isaac Sim without a GUI window.",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    config_path = Path(args.config).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"找不到机器人配置文件: {config_path}")

    config = _load_config(config_path)
    _validate_config(config)
    output_stage = _resolve_output_path(config_path, args.save_path)
    manifest_path = output_stage.with_suffix(".manifest.json")
    params_path = output_stage.with_suffix(".planner_params.yaml")

    try:
        from isaacsim import SimulationApp
    except ImportError:
        try:
            from omni.isaac.kit import SimulationApp
        except ImportError as exc:
            raise RuntimeError(
                "未检测到 Isaac Sim Python 环境，请用 Isaac Sim 自带的 python.sh 运行此脚本。"
            ) from exc

    app = SimulationApp({"headless": args.headless})
    try:
        from isaacsim.core.utils.extensions import enable_extension
        from isaacsim.storage.native import get_assets_root_path

        enable_extension("isaacsim.ros2.bridge")
        app.update()

        assets_root_path = args.assets_root or config.get("assets_root_path")
        if assets_root_path:
            _log(f"[nav_scene] 使用显式 assets_root_path: {assets_root_path}")
        else:
            _log("[nav_scene] 正在解析 Isaac assets_root_path ...")
            assets_root_path = get_assets_root_path()
            _log(f"[nav_scene] 已解析 assets_root_path: {assets_root_path}")

        source_stage = _resolve_source_stage(config, assets_root_path)
        manifest = _build_scene(
            app=app,
            config=config,
            source_stage=source_stage,
            output_stage=output_stage,
            assets_root_path=assets_root_path,
            headless=args.headless,
        )
        _write_manifest(manifest_path, manifest)
        _write_params_yaml(params_path, manifest)
        _log(f"[nav_scene] 清单文件已保存: {manifest_path}")
        _log(f"[nav_scene] cvrp 参数模板已保存: {params_path}")
    finally:
        app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
