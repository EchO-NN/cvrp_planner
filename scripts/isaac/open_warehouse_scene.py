#!/usr/bin/env python3
"""Open a generated warehouse USD in Isaac Sim and bind the viewport camera."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Open a warehouse USD in Isaac Sim with a visible viewport camera.",
    )
    parser.add_argument(
        "--stage",
        default=None,
        help="USD stage path to open.",
    )
    parser.add_argument(
        "--camera",
        default="/World/Cameras/Overview",
        help="Camera prim path to bind to the active viewport.",
    )
    parser.add_argument(
        "--updates",
        type=int,
        default=60,
        help="Number of app.update() ticks to wait before binding the viewport camera.",
    )
    parser.add_argument(
        "--enable-ros-bridge",
        action="store_true",
        help="Enable Isaac's ROS 2 Bridge extension before opening the stage.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        help="Start Isaac playback automatically after opening the stage.",
    )
    parser.add_argument(
        "--publish-without-verification",
        action="store_true",
        help="Force Isaac ROS 2 publishers to start without waiting for subscriber verification.",
    )
    return parser.parse_args()


def _default_stage_path() -> Path:
    generated_dir = Path(__file__).resolve().parent / "generated"
    candidates = [
        generated_dir / "full_warehouse_nav_robots.example.usd",
        generated_dir / "full_warehouse_robots.example.usd",
        generated_dir / "warehouse_demo.usd",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return candidates[0]


def _enable_extensions(extension_names: Iterable[str]) -> None:
    try:
        from isaacsim.core.utils.extensions import enable_extension
    except ImportError:
        from omni.isaac.core.utils.extensions import enable_extension

    for extension_name in extension_names:
        try:
            enable_extension(extension_name)
            print(f"[open_scene] 已启用扩展: {extension_name}", flush=True)
        except Exception as exc:
            print(f"[open_scene] 扩展启用失败，忽略: {extension_name}: {exc}", flush=True)


def _discover_robot_entries(stage) -> list[dict]:
    robots_root = stage.GetPrimAtPath("/Robots")
    if not robots_root.IsValid():
        return []

    entries = []
    for prim in robots_root.GetChildren():
        if not prim.IsValid():
            continue
        prim_path = prim.GetPath().pathString
        namespace_attr = prim.GetAttribute("robotNamespace")
        namespace = namespace_attr.Get() if namespace_attr and namespace_attr.HasValue() else prim.GetName()
        asset_prim_path = f"{prim_path}/Asset"
        asset_prim = stage.GetPrimAtPath(asset_prim_path)
        chassis_prim_path = _find_best_chassis_prim_path(stage, asset_prim_path if asset_prim.IsValid() else prim_path)
        entries.append(
            {
                "name": prim.GetName(),
                "namespace": str(namespace).strip("/") if namespace else prim.GetName(),
                "prim_path": prim_path,
                "asset_prim_path": asset_prim_path,
                "chassis_prim_path": chassis_prim_path,
            }
        )
    return entries


def _find_best_chassis_prim_path(stage, root_path: str) -> str:
    from pxr import Usd, UsdPhysics

    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return root_path

    preferred_name_tokens = (
        "chassis",
        "base_link",
        "base",
        "root_link",
        "body",
    )
    preferred_candidates: list[tuple[int, str]] = []
    fallback_candidates: list[str] = []

    for prim in Usd.PrimRange(root_prim):
        if not prim.IsValid():
            continue
        path = prim.GetPath().pathString
        name = prim.GetName().lower()
        has_rigid = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        has_articulation = prim.HasAPI(UsdPhysics.ArticulationRootAPI)
        if not (has_rigid or has_articulation):
            continue

        fallback_candidates.append(path)
        score = 0
        if has_articulation:
            score += 10
        if has_rigid:
            score += 5
        for index, token in enumerate(preferred_name_tokens):
            if token in name:
                score += 20 - index
                break
        preferred_candidates.append((score, path))

    if preferred_candidates:
        preferred_candidates.sort(key=lambda item: (-item[0], len(item[1])))
        return preferred_candidates[0][1]
    if fallback_candidates:
        fallback_candidates.sort(key=len)
        return fallback_candidates[0]
    return root_path


def _make_usd_path_value(path_text: str):
    try:
        import usdrt

        return usdrt.Sdf.Path(path_text)
    except Exception:
        from pxr import Sdf

        return Sdf.Path(path_text)


def _attach_runtime_odometry_graphs(stage) -> None:
    import omni.graph.core as og

    robot_entries = _discover_robot_entries(stage)
    if not robot_entries:
        print("[open_scene] 场景里没有 /Robots，跳过运行时 odom 图挂载", flush=True)
        return

    keys = og.Controller.Keys
    for entry in robot_entries:
        graph_path = f"/Graph/{entry['name']}_ROS_Odometry"
        if stage.GetPrimAtPath(graph_path).IsValid():
            print(f"[open_scene] 已存在运行时 odom 图，跳过: {graph_path}", flush=True)
            continue

        chassis_path = entry["chassis_prim_path"]
        namespace = entry["namespace"]
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                        ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                        ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                    ],
                    keys.SET_VALUES: [
                        ("ComputeOdometry.inputs:chassisPrim", [_make_usd_path_value(chassis_path)]),
                        ("PublishOdometry.inputs:topicName", "chassis/odom"),
                        ("PublishOdometry.inputs:nodeNamespace", namespace),
                        ("PublishOdometry.inputs:odomFrameId", "odom"),
                        ("PublishOdometry.inputs:chassisFrameId", "base_link"),
                        ("PublishOdometry.inputs:publishRawVelocities", False),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                        ("ComputeOdometry.outputs:execOut", "PublishOdometry.inputs:execIn"),
                        ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                        ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                        ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                        ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                        ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),
                        ("Context.outputs:context", "PublishOdometry.inputs:context"),
                    ],
                },
            )
            print(
                f"[open_scene] 已挂载运行时 odom 图: {graph_path} -> /{namespace}/chassis/odom "
                f"(chassis_prim={chassis_path})",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[open_scene] 运行时 odom 图挂载失败: {entry['name']} ({chassis_path}): {exc}",
                flush=True,
            )


def _attach_runtime_cmd_vel_graphs(stage) -> None:
    import omni.graph.core as og

    robot_entries = _discover_robot_entries(stage)
    if not robot_entries:
        print("[open_scene] 场景里没有 /Robots，跳过运行时 cmd_vel 图挂载", flush=True)
        return

    keys = og.Controller.Keys
    for entry in robot_entries:
        graph_path = f"/Graph/{entry['name']}_ROS_Drive"
        if stage.GetPrimAtPath(graph_path).IsValid():
            print(f"[open_scene] 已存在运行时 cmd_vel 图，跳过: {graph_path}", flush=True)
            continue

        namespace = entry["namespace"]
        robot_path = entry["asset_prim_path"]
        try:
            og.Controller.edit(
                {"graph_path": graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                        ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                        ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                        ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                        ("DifferentialController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                        ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                    ],
                    keys.SET_VALUES: [
                        ("SubscribeTwist.inputs:topicName", "cmd_vel"),
                        ("SubscribeTwist.inputs:nodeNamespace", namespace),
                        ("DifferentialController.inputs:wheelRadius", 0.14),
                        ("DifferentialController.inputs:wheelDistance", 0.4132),
                        (
                            "ArticulationController.inputs:jointNames",
                            ["joint_wheel_left", "joint_wheel_right"],
                        ),
                        ("ArticulationController.inputs:robotPath", robot_path),
                    ],
                    keys.CONNECT: [
                        ("Context.outputs:context", "SubscribeTwist.inputs:context"),
                        ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                        ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                        ("SubscribeTwist.outputs:execOut", "DifferentialController.inputs:execIn"),
                        ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                        ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                        ("BreakLinVel.outputs:x", "DifferentialController.inputs:linearVelocity"),
                        ("BreakAngVel.outputs:z", "DifferentialController.inputs:angularVelocity"),
                        (
                            "DifferentialController.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand",
                        ),
                    ],
                },
            )
            print(
                f"[open_scene] 已挂载运行时 cmd_vel 图: {graph_path} -> /{namespace}/cmd_vel "
                f"(robot_path={robot_path})",
                flush=True,
            )
        except Exception as exc:
            print(
                f"[open_scene] 运行时 cmd_vel 图挂载失败: {entry['name']} ({robot_path}): {exc}",
                flush=True,
            )


def main() -> int:
    args = _parse_args()
    stage_path = Path(args.stage).expanduser().resolve() if args.stage else _default_stage_path().resolve()
    if not stage_path.is_file():
        raise FileNotFoundError(f"找不到 USD 场景文件: {stage_path}")

    try:
        from isaacsim import SimulationApp  # Isaac Sim 4.5+
    except ImportError:
        try:
            from omni.isaac.kit import SimulationApp  # Isaac Sim 4.2/4.1
        except ImportError as exc:
            raise RuntimeError(
                "未检测到 Isaac Sim Python 环境，请用 Isaac Sim 自带的 python.sh 运行此脚本。"
            ) from exc

    app = SimulationApp({"headless": False})
    try:
        import carb
        import omni.usd
        import omni.timeline
        from omni.kit.viewport.utility import get_active_viewport

        if args.enable_ros_bridge:
            _enable_extensions(
                [
                    "isaacsim.core.nodes",
                    "isaacsim.ros2.bridge",
                    "isaacsim.sensors.physx",
                    "isaacsim.sensors.rtx",
                    "isaacsim.robot.wheeled_robots",
                ]
            )
            if args.publish_without_verification:
                carb.settings.get_settings().set_bool(
                    "/exts/isaacsim.ros2.bridge/publish_without_verification",
                    True,
                )
            for _ in range(10):
                app.update()

        context = omni.usd.get_context()
        if not context.open_stage(str(stage_path)):
            raise RuntimeError(f"无法打开 USD 场景: {stage_path}")

        for _ in range(max(args.updates, 1)):
            app.update()

        if args.enable_ros_bridge:
            _attach_runtime_odometry_graphs(context.get_stage())
            _attach_runtime_cmd_vel_graphs(context.get_stage())
            for _ in range(10):
                app.update()

        viewport = get_active_viewport()
        if viewport is None:
            raise RuntimeError("未找到活动 viewport，无法切换相机。")

        stage = context.get_stage()
        if stage.GetPrimAtPath(args.camera).IsValid():
            viewport.camera_path = args.camera
            print(f"[open_scene] 已切换到相机: {args.camera}", flush=True)
        else:
            print(f"[open_scene] 未找到相机 prim，保留当前视角: {args.camera}", flush=True)

        if args.play:
            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            for _ in range(max(args.updates, 1)):
                app.update()
            print("[open_scene] 已自动点击 Play", flush=True)

        print(f"[open_scene] 已打开场景: {stage_path}", flush=True)
        if args.enable_ros_bridge:
            print("[open_scene] 已启用 ROS 2 Bridge", flush=True)
        if args.publish_without_verification:
            print("[open_scene] 已启用 publish_without_verification", flush=True)

        while app.is_running():
            app.update()
    finally:
        app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
