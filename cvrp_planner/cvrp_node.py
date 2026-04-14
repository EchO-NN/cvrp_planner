import ast
import json
import math
import os
import shutil
import subprocess
import sys
import time
import xml.dom.minidom
from functools import partial
from xml.etree.ElementTree import Element, SubElement, tostring

import cv2
import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from matplotlib.collections import LineCollection
from nav_msgs.msg import Odometry
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from cvrp_planner.map_loader import load_map_config

try:
    from nav2_msgs.action import FollowWaypoints
except ImportError:  # pragma: no cover - depends on ROS installation
    FollowWaypoints = None


NUM_SAMPLES = 1000
MAX_DISTANCE = 120
DEFAULT_INFLATE_SIZE = 10
DEFAULT_CALCULATING_TIME = 10
COEFFICIENT = 10000
BACK_DISTANCE = 1.0


def _parse_parameter_list(value, name):
    if isinstance(value, str):
        stripped = value.strip()
        if not stripped:
            return []
        try:
            parsed = ast.literal_eval(stripped)
        except (ValueError, SyntaxError) as exc:
            raise ValueError(f'参数 {name} 解析失败: {exc}') from exc
    else:
        parsed = value

    if not isinstance(parsed, list):
        raise ValueError(f'参数 {name} 必须是列表')
    return parsed


def _transform_relative_pose_to_world(local_xy, origin_xy, origin_yaw_deg):
    yaw_rad = math.radians(origin_yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    rotation = np.array(
        [
            [cos_yaw, -sin_yaw],
            [sin_yaw, cos_yaw],
        ],
        dtype=float,
    )
    return np.array(origin_xy, dtype=float) + rotation @ np.array(local_xy, dtype=float)


class CVRPPlannerNode(Node):
    def __init__(self):
        super().__init__('cvrp_planner_node')
        self.get_logger().info('CVRP Planner Node 已启动')

        self.solution_pub = self.create_publisher(String, '/cvrp_solution', 10)
        self.base_dir = os.path.abspath(os.path.dirname(__file__))
        self.graph = nx.Graph()

        self.pose_subscriptions = []
        self.latest_robot_poses = {}
        self.action_clients = {}
        self.pending_dispatches = set()
        self.solve_started = False
        self.waiting_since = time.monotonic()
        self.last_wait_log_time = 0.0
        self.preview_figure = None

        self.declare_parameters(
            namespace='',
            parameters=[
                ('real_length', 50.0),
                ('origin_pixel', '[0, 0]'),
                ('origin_world', '[0.0, 0.0]'),
                ('robot_names', '[]'),
                ('robot_pose_topics', '[]'),
                ('robot_pose_type', 'pose_with_covariance_stamped'),
                ('robot_starts', '[]'),
                ('use_robot_starts_as_world_pose', False),
                ('odometry_is_relative', False),
                ('odometry_origin_positions', '[]'),
                ('odometry_origin_yaws_deg', '[]'),
                ('customers', '[]'),
                ('demands', '[]'),
                ('num_vehicles', 0),
                ('vehicle_capacities', '[]'),
                ('follow_waypoints_actions', '[]'),
                ('auto_dispatch', True),
                ('calculating_time', DEFAULT_CALCULATING_TIME),
                ('inflate_size', DEFAULT_INFLATE_SIZE),
                ('inflation_size', 0),
                ('map_path', ''),
                ('frame_id', 'map'),
                ('pose_wait_timeout', 10.0),
                ('nav2_wait_timeout', 15.0),
                ('visualize', False),
            ],
        )

        self._load_static_config()
        self._setup_pose_subscribers()

        self.start_timer = self.create_timer(0.5, self._maybe_start_planning)

    def _load_static_config(self):
        try:
            self.real_length = float(self.get_parameter('real_length').value)
            self.origin_pixel = np.array(
                _parse_parameter_list(self.get_parameter('origin_pixel').value, 'origin_pixel'),
                dtype=float,
            )
            self.origin_world = np.array(
                _parse_parameter_list(self.get_parameter('origin_world').value, 'origin_world'),
                dtype=float,
            )

            robot_names = _parse_parameter_list(self.get_parameter('robot_names').value, 'robot_names')
            self.fallback_robot_starts_world = np.array(
                _parse_parameter_list(self.get_parameter('robot_starts').value, 'robot_starts'),
                dtype=float,
            ) if self.get_parameter('robot_starts').value else np.empty((0, 2))

            if not robot_names:
                if len(self.fallback_robot_starts_world) > 0:
                    robot_names = [f'robot_{index}' for index in range(len(self.fallback_robot_starts_world))]
                else:
                    num_vehicles = int(self.get_parameter('num_vehicles').value)
                    robot_names = [f'robot_{index}' for index in range(num_vehicles)]
            self.robot_names = [str(name).strip('/') for name in robot_names]

            self.customers_world = np.array(
                _parse_parameter_list(self.get_parameter('customers').value, 'customers'),
                dtype=float,
            )
            self.demands = [
                int(value)
                for value in _parse_parameter_list(self.get_parameter('demands').value, 'demands')
            ]
            self.vehicle_capacities = [
                int(value)
                for value in _parse_parameter_list(
                    self.get_parameter('vehicle_capacities').value,
                    'vehicle_capacities',
                )
            ]

            pose_topics = _parse_parameter_list(
                self.get_parameter('robot_pose_topics').value,
                'robot_pose_topics',
            )
            self.pose_type = self.get_parameter('robot_pose_type').value
            self.use_robot_starts_as_world_pose = bool(
                self.get_parameter('use_robot_starts_as_world_pose').value,
            )
            self.odometry_is_relative = bool(self.get_parameter('odometry_is_relative').value)
            if pose_topics:
                self.robot_pose_topics = [str(topic) for topic in pose_topics]
            else:
                if self.pose_type == 'pose_stamped':
                    default_suffix = 'pose'
                elif self.pose_type == 'odometry':
                    default_suffix = 'chassis/odom'
                else:
                    default_suffix = 'amcl_pose'
                self.robot_pose_topics = [
                    f'/{robot_name}/{default_suffix}'
                    for robot_name in self.robot_names
                ]

            odometry_origin_positions_raw = _parse_parameter_list(
                self.get_parameter('odometry_origin_positions').value,
                'odometry_origin_positions',
            )
            if odometry_origin_positions_raw:
                self.odometry_origin_positions = np.array(
                    odometry_origin_positions_raw,
                    dtype=float,
                )
            elif self.odometry_is_relative and len(self.fallback_robot_starts_world) == len(self.robot_names):
                self.odometry_origin_positions = np.array(self.fallback_robot_starts_world, dtype=float)
            else:
                self.odometry_origin_positions = np.empty((0, 2), dtype=float)

            odometry_origin_yaws_raw = _parse_parameter_list(
                self.get_parameter('odometry_origin_yaws_deg').value,
                'odometry_origin_yaws_deg',
            )
            if odometry_origin_yaws_raw:
                self.odometry_origin_yaws_deg = [float(value) for value in odometry_origin_yaws_raw]
            elif self.odometry_is_relative:
                self.odometry_origin_yaws_deg = [0.0] * len(self.robot_names)
            else:
                self.odometry_origin_yaws_deg = []

            follow_waypoints_actions = _parse_parameter_list(
                self.get_parameter('follow_waypoints_actions').value,
                'follow_waypoints_actions',
            )
            if follow_waypoints_actions:
                self.follow_waypoints_actions = [str(name) for name in follow_waypoints_actions]
            else:
                self.follow_waypoints_actions = [
                    f'/{robot_name}/follow_waypoints'
                    for robot_name in self.robot_names
                ]
        except Exception as exc:
            self.get_logger().fatal(f'参数转换错误: {exc}')
            raise

        self.num_vehicles = len(self.robot_names)
        self.calculating_time = int(self.get_parameter('calculating_time').value)
        self.inflate_size = int(self.get_parameter('inflate_size').value)
        if self.inflate_size <= 0:
            self.inflate_size = int(self.get_parameter('inflation_size').value)
        if self.inflate_size <= 0:
            self.inflate_size = DEFAULT_INFLATE_SIZE

        self.auto_dispatch = bool(self.get_parameter('auto_dispatch').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.pose_wait_timeout = float(self.get_parameter('pose_wait_timeout').value)
        self.nav2_wait_timeout = float(self.get_parameter('nav2_wait_timeout').value)
        self.visualize = bool(self.get_parameter('visualize').value)

        if self.customers_world.size == 0:
            raise ValueError('customers 不能为空')
        if len(self.demands) != len(self.customers_world):
            raise ValueError('demands 数量必须与 customers 一致')
        if len(self.vehicle_capacities) != self.num_vehicles:
            raise ValueError('vehicle_capacities 数量必须与机器人数量一致')
        if len(self.robot_pose_topics) != self.num_vehicles:
            raise ValueError('robot_pose_topics 数量必须与机器人数量一致')
        if len(self.follow_waypoints_actions) != self.num_vehicles:
            raise ValueError('follow_waypoints_actions 数量必须与机器人数量一致')
        if len(self.fallback_robot_starts_world) not in (0, self.num_vehicles):
            raise ValueError('robot_starts 数量必须为 0 或与机器人数量一致')
        if self.odometry_is_relative:
            if self.pose_type != 'odometry':
                raise ValueError('odometry_is_relative=true 时 robot_pose_type 必须为 odometry')
            if len(self.odometry_origin_positions) != self.num_vehicles:
                raise ValueError('odometry_origin_positions 数量必须与机器人数量一致')
            if len(self.odometry_origin_yaws_deg) != self.num_vehicles:
                raise ValueError('odometry_origin_yaws_deg 数量必须与机器人数量一致')

        map_path = self.get_parameter('map_path').value
        if not map_path:
            map_path = os.path.join(self.base_dir, 'maps', 'manual_map_from_MAP.yaml')
        self.get_logger().info(f'加载地图: {map_path}')
        map_config = load_map_config(map_path)
        self.map_img = map_config['image']
        self.height_px = map_config['height_px']
        self.width_px = map_config['width_px']

        if map_config['derived_from_yaml']:
            self.real_length = float(map_config['real_length'])
            self.origin_pixel = np.array(map_config['origin_pixel'], dtype=float)
            self.origin_world = np.array(map_config['origin_world'], dtype=float)
            self.frame_id = map_config['frame_id']
            self.get_logger().info(
                f'从 ROS map YAML 派生地图参数: real_length={self.real_length:.3f}, '
                f'origin_world={self.origin_world.tolist()}',
            )

        self.meters_per_pixel = self.real_length / self.width_px

        _, binary = cv2.threshold(self.map_img, 127, 255, cv2.THRESH_BINARY)
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.inflate_size, self.inflate_size),
        )
        self.inflated_map = cv2.erode(binary, kernel, iterations=1)
        self.sample_pts = self.sample_free_points(NUM_SAMPLES)

    def _setup_pose_subscribers(self):
        for robot_name, topic in zip(self.robot_names, self.robot_pose_topics):
            callback = self._make_pose_callback(robot_name)
            if self.pose_type == 'pose_stamped':
                subscription = self.create_subscription(PoseStamped, topic, callback, 10)
            elif self.pose_type == 'odometry':
                subscription = self.create_subscription(Odometry, topic, callback, 10)
            else:
                subscription = self.create_subscription(PoseWithCovarianceStamped, topic, callback, 10)
            self.pose_subscriptions.append(subscription)
            self.latest_robot_poses[robot_name] = None
            self.get_logger().info(f'订阅机器人 {robot_name} 位姿: {topic}')

        if self.pose_type == 'odometry' and self.odometry_is_relative:
            self.get_logger().info('将把局部 odometry 通过出生点和初始朝向转换为仓库全局坐标')
        if self.use_robot_starts_as_world_pose and len(self.fallback_robot_starts_world) == self.num_vehicles:
            for index, robot_name in enumerate(self.robot_names):
                self.latest_robot_poses[robot_name] = np.array(
                    self.fallback_robot_starts_world[index],
                    dtype=float,
                )
            self.get_logger().warn('当前配置将 robot_starts 视为世界坐标真值，暂时忽略实时位姿覆盖')

    def _odom_to_world(self, robot_name, local_xy):
        if not self.odometry_is_relative:
            return np.array(local_xy, dtype=float)

        robot_index = self.robot_names.index(robot_name)
        return _transform_relative_pose_to_world(
            local_xy,
            self.odometry_origin_positions[robot_index],
            self.odometry_origin_yaws_deg[robot_index],
        )

    def _make_pose_callback(self, robot_name):
        def callback(msg):
            if self.use_robot_starts_as_world_pose:
                return
            if isinstance(msg, PoseStamped):
                pose = msg.pose
            elif isinstance(msg, Odometry):
                pose = msg.pose.pose
            else:
                pose = msg.pose.pose
            position_xy = np.array([pose.position.x, pose.position.y], dtype=float)
            if isinstance(msg, Odometry):
                position_xy = self._odom_to_world(robot_name, position_xy)
            self.latest_robot_poses[robot_name] = position_xy
        return callback

    def _maybe_start_planning(self):
        if self.solve_started:
            return

        if self.use_robot_starts_as_world_pose and len(self.fallback_robot_starts_world) == self.num_vehicles:
            self.start_timer.cancel()
            self.solve_started = True
            self.get_logger().info('已使用 robot_starts 作为世界坐标真值，开始规划')
            self.solve_and_dispatch(self.fallback_robot_starts_world)
            return

        ready_names = [
            name
            for name in self.robot_names
            if self.latest_robot_poses.get(name) is not None
        ]
        if len(ready_names) == self.num_vehicles:
            self.start_timer.cancel()
            self.solve_started = True
            robot_starts_world = np.array(
                [self.latest_robot_poses[name] for name in self.robot_names],
                dtype=float,
            )
            self.get_logger().info('已收到全部机器人实时位姿，开始规划')
            self.solve_and_dispatch(robot_starts_world)
            return

        elapsed = time.monotonic() - self.waiting_since
        if elapsed - self.last_wait_log_time >= 2.0:
            missing = [name for name in self.robot_names if self.latest_robot_poses.get(name) is None]
            self.get_logger().info(f'等待机器人位姿: 缺少 {missing}')
            self.last_wait_log_time = elapsed

        if elapsed < self.pose_wait_timeout:
            return

        if len(self.fallback_robot_starts_world) == self.num_vehicles:
            self.start_timer.cancel()
            self.solve_started = True
            self.get_logger().warn('实时位姿等待超时，回退到 YAML 中的 robot_starts')
            self.solve_and_dispatch(self.fallback_robot_starts_world)
            return

        self.get_logger().error('实时位姿等待超时，且没有可用的 robot_starts 回退数据')
        self.start_timer.cancel()
        rclpy.shutdown()

    def world_to_pixel(self, world_pt):
        x, y = world_pt[0], world_pt[1]
        u0, v0 = self.origin_pixel[0], self.origin_pixel[1]
        x0, y0 = self.origin_world[0], self.origin_world[1]
        u = u0 + (x - x0) / self.meters_per_pixel
        v = v0 - (y - y0) / self.meters_per_pixel
        return np.array([u, v], dtype=float)

    def pixel_to_world(self, pixel_pt):
        u, v = pixel_pt[0], pixel_pt[1]
        u0, v0 = self.origin_pixel[0], self.origin_pixel[1]
        x0, y0 = self.origin_world[0], self.origin_world[1]
        x = x0 + (u - u0) * self.meters_per_pixel
        y = y0 - (v - v0) * self.meters_per_pixel
        return np.array([x, y], dtype=float)

    def is_free(self, point):
        x, y = int(point[0]), int(point[1])
        return (
            0 <= x < self.width_px
            and 0 <= y < self.height_px
            and self.inflated_map[y, x] == 255
        )

    def sample_free_points(self, count):
        points = []
        attempts = 0
        while len(points) < count and attempts < 100000:
            point = np.random.uniform([0, 0], [self.width_px, self.height_px])
            if self.is_free(point):
                points.append(point)
            attempts += 1
        if len(points) < count:
            self.get_logger().warn(f'仅采样到 {len(points)} 个自由点，目标 {count} 个')
        return np.array(points, dtype=float)

    def collision_free(self, point_a, point_b):
        for ratio in np.linspace(0, 1, 100):
            if not self.is_free(point_a + ratio * (point_b - point_a)):
                return False
        return True

    def solve_and_dispatch(self, robot_starts_world):
        self.robot_starts_world = np.array(robot_starts_world, dtype=float)
        self.robot_starts = np.array(
            [self.world_to_pixel(point) for point in self.robot_starts_world],
            dtype=float,
        )
        self.customers = np.array(
            [self.world_to_pixel(point) for point in self.customers_world],
            dtype=float,
        )

        vehicle_paths_info = self._solve_vehicle_paths()
        if vehicle_paths_info is None:
            rclpy.shutdown()
            return

        vehicle_paths_px = [[coord for coord, _ in path] for path in vehicle_paths_info]
        self.visualize_solution(vehicle_paths_px)

        waypoint_payload = self._build_waypoints(vehicle_paths_info)
        self._write_solution_xml(waypoint_payload)
        self._publish_solution_summary(waypoint_payload)

        if not self.auto_dispatch:
            self.get_logger().info('auto_dispatch=false，仅输出规划结果，不调用 Nav2')
            rclpy.shutdown()
            return

        self._dispatch_to_nav2(waypoint_payload)

    def _solve_vehicle_paths(self):
        vehicle_count = self.num_vehicles
        task_count = len(self.customers)
        non_dummy_nodes = vehicle_count + task_count
        total_nodes = non_dummy_nodes + vehicle_count

        combined = np.vstack([self.sample_pts, self.robot_starts, self.customers])
        base_index = len(self.sample_pts)

        self.graph.clear()
        for node_index, coord in enumerate(combined):
            self.graph.add_node(node_index, pos=coord)

        for first in range(len(combined)):
            for second in range(first + 1, len(combined)):
                distance = np.linalg.norm(combined[first] - combined[second])
                if distance <= MAX_DISTANCE and self.collision_free(combined[first], combined[second]):
                    self.graph.add_edge(first, second, weight=distance)

        distance_matrix = np.full((non_dummy_nodes, non_dummy_nodes), 1e9)
        for row in range(non_dummy_nodes):
            for col in range(non_dummy_nodes):
                if row == col:
                    distance_matrix[row, col] = 0
                    continue
                try:
                    distance_matrix[row, col] = nx.shortest_path_length(
                        self.graph,
                        base_index + row,
                        base_index + col,
                        weight='weight',
                    )
                except nx.NetworkXNoPath:
                    pass

        full_distance = np.full((total_nodes, total_nodes), int(1e9))
        full_distance[:non_dummy_nodes, :non_dummy_nodes] = distance_matrix.astype(int)
        for node_index in range(non_dummy_nodes):
            for vehicle_id in range(vehicle_count):
                full_distance[node_index, non_dummy_nodes + vehicle_id] = 0
        for vehicle_id in range(vehicle_count):
            full_distance[non_dummy_nodes + vehicle_id, non_dummy_nodes + vehicle_id] = 0

        all_demands = [0] * vehicle_count + self.demands + [0] * vehicle_count

        manager = pywrapcp.RoutingIndexManager(
            total_nodes,
            vehicle_count,
            list(range(vehicle_count)),
            list(range(non_dummy_nodes, total_nodes)),
        )
        routing = pywrapcp.RoutingModel(manager)

        def dist_cb(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            return int(full_distance[from_node, to_node])

        time_callback_index = routing.RegisterTransitCallback(dist_cb)

        def zero_cb(from_index, to_index):
            del from_index, to_index
            return 0

        zero_callback_index = routing.RegisterTransitCallback(zero_cb)
        routing.SetArcCostEvaluatorOfAllVehicles(zero_callback_index)

        max_time_horizon = int(total_nodes * MAX_DISTANCE)
        routing.AddDimension(
            time_callback_index,
            0,
            max_time_horizon,
            True,
            'Time',
        )
        time_dimension = routing.GetDimensionOrDie('Time')
        time_dimension.SetGlobalSpanCostCoefficient(COEFFICIENT)

        def demand_cb(index):
            return all_demands[manager.IndexToNode(index)]

        demand_callback_index = routing.RegisterUnaryTransitCallback(demand_cb)
        routing.AddDimensionWithVehicleCapacity(
            demand_callback_index,
            0,
            self.vehicle_capacities,
            True,
            'Capacity',
        )

        search_params = pywrapcp.DefaultRoutingSearchParameters()
        search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
        search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
        search_params.time_limit.seconds = self.calculating_time

        solution = routing.SolveWithParameters(search_params)
        if not solution:
            self.get_logger().error('未找到可行解')
            return None

        max_time = 0
        for vehicle_id in range(self.num_vehicles):
            index = routing.End(vehicle_id)
            route_time = solution.Value(time_dimension.CumulVar(index))
            max_time = max(max_time, route_time)
        self.get_logger().info(f'最长路径代价: {max_time}')

        vehicle_paths_info = []
        for vehicle_id in range(vehicle_count):
            path_info = []
            index = routing.Start(vehicle_id)
            previous_prm_node = None

            while True:
                current_node = manager.IndexToNode(index)
                if current_node < non_dummy_nodes:
                    prm_node = base_index + current_node
                    current_coord = combined[prm_node]
                    if previous_prm_node is not None:
                        try:
                            path_nodes = nx.shortest_path(
                                self.graph,
                                previous_prm_node,
                                prm_node,
                                weight='weight',
                            )
                            for path_node in path_nodes[1:]:
                                coord = self.graph.nodes[path_node]['pos']
                                path_info.append((coord, path_node))
                        except nx.NetworkXNoPath:
                            self.get_logger().error(
                                f'车辆 {vehicle_id} 无法找到路径: {previous_prm_node}->{prm_node}',
                            )
                            path_info.append((current_coord, prm_node))
                    else:
                        path_info.append((current_coord, prm_node))
                    previous_prm_node = prm_node

                if routing.IsEnd(index):
                    break
                index = solution.Value(routing.NextVar(index))

            vehicle_paths_info.append(path_info)

        return vehicle_paths_info

    def _build_waypoints(self, vehicle_paths_info):
        payload = []
        customer_start_index = len(self.sample_pts) + self.num_vehicles
        customer_end_index = customer_start_index + len(self.customers) - 1

        for vehicle_id, robot_name in enumerate(self.robot_names):
            path_info = vehicle_paths_info[vehicle_id]
            customer_points = []
            for index, (coord, node_index) in enumerate(path_info):
                if customer_start_index <= node_index <= customer_end_index:
                    if index > 0:
                        previous_coord, _ = path_info[index - 1]
                    else:
                        previous_coord = coord
                    customer_points.append((coord, previous_coord))

            waypoints = []
            for point_index, (customer_px, previous_px) in enumerate(customer_points, start=1):
                customer_world = self.pixel_to_world(customer_px)
                previous_world = self.pixel_to_world(previous_px)

                direction = customer_world - previous_world
                distance = np.linalg.norm(direction)
                if distance > 0:
                    approach_point = customer_world - (BACK_DISTANCE * direction / distance)
                else:
                    approach_point = customer_world

                angle = math.atan2(direction[1], direction[0])
                orientation_z = math.sin(angle / 2)
                orientation_w = math.cos(angle / 2)

                waypoint = {
                    'index': point_index,
                    'target': customer_world.tolist(),
                    'approach': approach_point.tolist(),
                    'quat_z': orientation_z,
                    'quat_w': orientation_w,
                }
                waypoints.append(waypoint)

            payload.append(
                {
                    'vehicle_id': vehicle_id,
                    'robot_name': robot_name,
                    'action_name': self.follow_waypoints_actions[vehicle_id],
                    'waypoints': waypoints,
                },
            )
        return payload

    def _write_solution_xml(self, waypoint_payload):
        root = Element('Maypoints')
        for vehicle in waypoint_payload:
            vehicle_elem = SubElement(
                root,
                'Vehicle',
                {
                    'id': str(vehicle['vehicle_id']),
                    'name': vehicle['robot_name'],
                },
            )
            for waypoint in vehicle['waypoints']:
                maypoint = SubElement(vehicle_elem, 'Maypoint')
                SubElement(maypoint, 'Name').text = str(waypoint['index'])
                SubElement(maypoint, 'Pos_x').text = f"{waypoint['approach'][0]:.5f}"
                SubElement(maypoint, 'Pos_y').text = f"{waypoint['approach'][1]:.5f}"
                SubElement(maypoint, 'Pos_z').text = '0'
                SubElement(maypoint, 'Ort_x').text = '0'
                SubElement(maypoint, 'Ort_y').text = '0'
                SubElement(maypoint, 'Ort_z').text = f"{waypoint['quat_z']:.6f}"
                SubElement(maypoint, 'Ort_w').text = f"{waypoint['quat_w']:.6f}"

        xml_str = xml.dom.minidom.parseString(tostring(root)).toprettyxml(indent='    ')
        output_path = os.path.join(self.base_dir, 'params', 'cvrp_solution.xml')
        with open(output_path, 'w', encoding='utf-8') as file_obj:
            file_obj.write(xml_str)
        self.get_logger().info(f'结果已保存至 {output_path}')

    def _publish_solution_summary(self, waypoint_payload):
        message = String()
        message.data = json.dumps(waypoint_payload, ensure_ascii=False)
        self.solution_pub.publish(message)

    def _build_pose_stamped(self, waypoint):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.frame_id
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = float(waypoint['approach'][0])
        pose_stamped.pose.position.y = float(waypoint['approach'][1])
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = float(waypoint['quat_z'])
        pose_stamped.pose.orientation.w = float(waypoint['quat_w'])
        return pose_stamped

    def _dispatch_to_nav2(self, waypoint_payload):
        if FollowWaypoints is None:
            self.get_logger().error('当前环境缺少 nav2_msgs，无法调用 FollowWaypoints')
            rclpy.shutdown()
            return

        self.pending_dispatches = set()

        for vehicle in waypoint_payload:
            robot_name = vehicle['robot_name']
            poses = [self._build_pose_stamped(waypoint) for waypoint in vehicle['waypoints']]
            if not poses:
                self.get_logger().info(f'{robot_name} 没有分配到任务点，跳过派发')
                continue

            action_name = vehicle['action_name']
            client = ActionClient(self, FollowWaypoints, action_name)
            self.action_clients[robot_name] = client
            self.get_logger().info(f'等待 {robot_name} 的 Nav2 action: {action_name}')
            if not client.wait_for_server(timeout_sec=self.nav2_wait_timeout):
                self.get_logger().error(
                    f'{robot_name} 的 action server 不可用: {action_name}。'
                    '如果你要跑闭环，请先启动 cvrp_planner/isaac_multi_nav.launch.py '
                    '或 cvrp_planner/isaac_closed_loop.launch.py',
                )
                continue

            goal = FollowWaypoints.Goal()
            goal.poses = poses

            self.pending_dispatches.add(robot_name)
            send_goal_future = client.send_goal_async(goal)
            send_goal_future.add_done_callback(partial(self._handle_goal_response, robot_name))
            self.get_logger().info(f'已向 {robot_name} 下发 {len(poses)} 个任务点')

        if not self.pending_dispatches:
            self.get_logger().warn(
                '没有任何机器人成功进入执行阶段。当前通常意味着 Nav2 waypoint_follower '
                '还没有启动，或者 follow_waypoints action 名称与配置不一致。',
            )
            rclpy.shutdown()

    def _handle_goal_response(self, robot_name, future):
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - ROS runtime path
            self.get_logger().error(f'{robot_name} 发送 FollowWaypoints 失败: {exc}')
            self.pending_dispatches.discard(robot_name)
            self._shutdown_if_finished()
            return

        if not goal_handle.accepted:
            self.get_logger().error(f'{robot_name} 的 FollowWaypoints goal 被拒绝')
            self.pending_dispatches.discard(robot_name)
            self._shutdown_if_finished()
            return

        self.get_logger().info(f'{robot_name} 已接受任务，等待执行完成')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(partial(self._handle_goal_result, robot_name))

    def _handle_goal_result(self, robot_name, future):
        try:
            result = future.result().result
            missed = list(result.missed_waypoints)
            if missed:
                self.get_logger().warn(f'{robot_name} 执行完成，但遗漏航点: {missed}')
            else:
                self.get_logger().info(f'{robot_name} 已完成全部航点')
        except Exception as exc:  # pragma: no cover - ROS runtime path
            self.get_logger().error(f'{robot_name} 获取执行结果失败: {exc}')
        finally:
            self.pending_dispatches.discard(robot_name)
            self._shutdown_if_finished()

    def _shutdown_if_finished(self):
        if self.pending_dispatches:
            return
        self.get_logger().info('全部机器人任务流程结束，准备退出')
        rclpy.shutdown()

    def visualize_solution(self, vehicle_paths_px):
        if not self.visualize:
            return

        vis_map = cv2.cvtColor(self.inflated_map, cv2.COLOR_GRAY2RGB)
        vis_map[vis_map > 0] = 255

        figure = plt.figure(figsize=(12, 12))
        plt.imshow(vis_map, extent=[0, self.width_px, self.height_px, 0])

        base_segments = []
        for first, second in self.graph.edges():
            first_pos = self.graph.nodes[first].get('pos')
            second_pos = self.graph.nodes[second].get('pos')
            if first_pos is None or second_pos is None:
                continue
            base_segments.append([first_pos, second_pos])

        if base_segments:
            base_lines = LineCollection(
                base_segments,
                colors='#9A9A9A',
                linewidths=0.35,
                alpha=0.45,
                zorder=1,
            )
            plt.gca().add_collection(base_lines)

        if len(self.sample_pts) > 0:
            plt.scatter(
                self.sample_pts[:, 0],
                self.sample_pts[:, 1],
                c='#B0B0B0',
                s=2,
                alpha=0.35,
                zorder=2,
            )

        starts = np.array(self.robot_starts)
        customers = np.array(self.customers)
        plt.scatter(starts[:, 0], starts[:, 1], c='blue', s=100, label='Robots', zorder=5)
        plt.scatter(customers[:, 0], customers[:, 1], c='red', s=100, label='Tasks', zorder=5)

        colors = ['#FF0000', '#00AA00', '#0000FF', '#FF00FF', '#00AAAA']
        for vehicle_id, path in enumerate(vehicle_paths_px):
            if not path:
                continue
            color = colors[vehicle_id % len(colors)]
            path_array = np.array(path)
            plt.scatter(path_array[:, 0], path_array[:, 1], c=color, s=30, zorder=4)

            if len(path_array) > 1:
                segments = np.stack([path_array[:-1], path_array[1:]], axis=1)
                line_collection = LineCollection(segments, colors=color, linewidths=2, zorder=4)
                plt.gca().add_collection(line_collection)

                for index in range(0, len(path_array) - 1, 5):
                    dx = path_array[index + 1][0] - path_array[index][0]
                    dy = path_array[index + 1][1] - path_array[index][1]
                    plt.arrow(
                        path_array[index][0],
                        path_array[index][1],
                        dx * 0.8,
                        dy * 0.8,
                        head_width=5,
                        head_length=8,
                        fc=color,
                        ec=color,
                    )

        plt.title('CVRP Solution Visualization')
        plt.legend()
        plt.axis('equal')
        plt.xlim(0, self.width_px)
        plt.ylim(self.height_px, 0)
        plt.margins(0)
        plt.grid(False)
        plt.tight_layout()
        preview_paths = [os.path.join(self.base_dir, 'params', 'cvrp_strategy_preview.png')]

        workspace_dir = self._find_workspace_dir()
        if workspace_dir:
            source_preview = os.path.join(
                workspace_dir,
                'cvrp_planner',
                'params',
                'cvrp_strategy_preview.png',
            )
            if source_preview not in preview_paths:
                preview_paths.append(source_preview)

        for preview_path in preview_paths:
            os.makedirs(os.path.dirname(preview_path), exist_ok=True)
            figure.savefig(preview_path, dpi=150)
            self.get_logger().info(f'策略图预览已保存至 {preview_path}')

        self.preview_figure = figure
        primary_preview = preview_paths[-1]
        try:
            if self._open_preview_image(primary_preview):
                self.get_logger().info(f'已打开策略图 PNG 预览: {primary_preview}')
            else:
                self.get_logger().warn(f'系统图片查看器不可用，请直接打开: {primary_preview}')
        except Exception as exc:
            self.get_logger().warn(f'策略图 PNG 预览打开失败，但文件已保存: {exc}')

        plt.close(figure)

    def _open_preview_image(self, image_path):
        open_commands = []
        if shutil.which('xdg-open'):
            open_commands.append(['xdg-open', image_path])
        if shutil.which('gio'):
            open_commands.append(['gio', 'open', image_path])
        open_commands.append([sys.executable, '-m', 'cvrp_planner.preview_window', image_path])

        for command in open_commands:
            try:
                subprocess.Popen(
                    command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    start_new_session=True,
                )
                return True
            except Exception:
                continue
        return False

    def _find_workspace_dir(self):
        current = self.base_dir
        while True:
            if os.path.isfile(os.path.join(current, 'package.xml')) and os.path.isdir(
                os.path.join(current, 'cvrp_planner', 'params'),
            ):
                return current
            parent = os.path.dirname(current)
            if parent == current:
                return None
            current = parent


def main(args=None):
    rclpy.init(args=args)
    node = CVRPPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
