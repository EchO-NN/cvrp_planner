import json
import math
import os
import sys
import threading

import cv2
import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.parameter import Parameter

from cvrp_planner.map_loader import load_map_config

plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['WenQuanYi Zen Hei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False


def _parse_json_list(text, label):
    try:
        value = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(f'{label} 不是合法 JSON: {exc}') from exc
    if not isinstance(value, list):
        raise ValueError(f'{label} 必须是 JSON 列表')
    return value


def _yaml_quote(text):
    return "'" + str(text).replace("'", "''") + "'"


def _transform_relative_pose_to_world(local_xy, origin_xy, origin_yaw_deg):
    yaw_rad = math.radians(origin_yaw_deg)
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    return [
        float(origin_xy[0] + cos_yaw * local_xy[0] - sin_yaw * local_xy[1]),
        float(origin_xy[1] + sin_yaw * local_xy[0] + cos_yaw * local_xy[1]),
    ]


class CVRPPlannerGUINode(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'cvrp_planner_gui')
        QMainWindow.__init__(self)

        self.declare_parameter('robot_names', '["robot_0", "robot_1"]')
        self.declare_parameter('robot_pose_topics', '[]')
        self.declare_parameter('robot_pose_type', 'pose_with_covariance_stamped')
        self.declare_parameter('robot_starts', '[]')
        self.declare_parameter('use_robot_starts_as_world_pose', False)
        self.declare_parameter('odometry_is_relative', False)
        self.declare_parameter('odometry_origin_positions', '[]')
        self.declare_parameter('odometry_origin_yaws_deg', '[]')
        self.declare_parameter('customers', '[]')
        self.declare_parameter('demands', '[]')
        self.declare_parameter('num_vehicles', 0)
        self.declare_parameter('vehicle_capacities', '[]')
        self.declare_parameter('map_path', '')
        self.declare_parameter('calculating_time', 10)
        self.declare_parameter('inflate_size', 5)
        self.declare_parameter('inflation_size', 0)
        self.declare_parameter('real_length', 50.0)
        self.declare_parameter('origin_pixel', '[0, 0]')
        self.declare_parameter('origin_world', '[0.0, 0.0]')
        self.declare_parameter('auto_dispatch', True)
        self.declare_parameter('follow_waypoints_actions', '[]')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('pose_wait_timeout', 10.0)
        self.declare_parameter('nav2_wait_timeout', 15.0)
        self.declare_parameter('visualize', False)

        self.base_dir = os.path.abspath(os.path.dirname(__file__))
        self.map_path = ''
        self.map_img = None
        self.image_width = 0
        self.image_height = 0
        self.map_derived_from_yaml = False

        self.robot_names = []
        self.robot_pose_topics = []
        self.pose_subscriptions = []
        self.robot_poses = {}
        self.use_robot_starts_as_world_pose = False
        self.odometry_is_relative = False
        self.odometry_origin_positions = []
        self.odometry_origin_yaws_deg = []
        self.robot_pose_lock = threading.Lock()
        self.robot_state_dirty = False

        self.vehicle_capacity_defaults = []
        self.customers = []
        self.demands = []

        self.real_length = 0.0
        self.real_height = 0.0
        self.origin_pixel = [0, 0]
        self.origin_world = [0.0, 0.0]
        self.meters_per_pixel = 0.0
        self.map_extent = [0, 0, 0, 0]
        self.aspect_ratio = 1.0

        self.setWindowTitle('CVRP 任务点配置器')
        self._init_ui()
        self._load_initial_state()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_robot_widgets)
        self.refresh_timer.start(300)

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)

        self.fig = Figure(figsize=(6, 6))
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.canvas.mpl_connect('button_press_event', self.on_click)
        layout.addWidget(self.canvas, 2)

        panel = QVBoxLayout()
        layout.addLayout(panel, 1)

        panel.addWidget(QLabel('地图实际宽度(米):'))
        self.real_length_input = QLineEdit('50.0')
        panel.addWidget(self.real_length_input)

        panel.addWidget(QLabel('参考原点像素坐标 [u,v]:'))
        self.origin_pixel_input = QLineEdit('[0, 0]')
        panel.addWidget(self.origin_pixel_input)

        panel.addWidget(QLabel('参考原点世界坐标 [x,y]:'))
        self.origin_world_input = QLineEdit('[0.0, 0.0]')
        panel.addWidget(self.origin_world_input)

        btn_apply_map = QPushButton('应用地图参数')
        btn_apply_map.clicked.connect(self.apply_map_params)
        panel.addWidget(btn_apply_map)

        panel.addWidget(QLabel('机器人列表(JSON):'))
        self.robot_names_input = QLineEdit('["robot_0", "robot_1"]')
        panel.addWidget(self.robot_names_input)

        panel.addWidget(QLabel('机器人位姿话题(JSON，可留空自动推导):'))
        self.robot_topics_input = QLineEdit('[]')
        panel.addWidget(self.robot_topics_input)

        panel.addWidget(QLabel('位姿消息类型:'))
        self.pose_type_combo = QComboBox()
        self.pose_type_combo.addItems([
            'pose_with_covariance_stamped',
            'pose_stamped',
            'odometry',
        ])
        panel.addWidget(self.pose_type_combo)

        btn_apply_robots = QPushButton('应用机器人配置')
        btn_apply_robots.clicked.connect(self.apply_robot_config)
        panel.addWidget(btn_apply_robots)

        panel.addWidget(QLabel('机器人状态:'))
        self.robot_table = QTableWidget(0, 5)
        self.robot_table.setHorizontalHeaderLabels(['机器人', '话题', '状态', 'X', 'Y'])
        panel.addWidget(self.robot_table)

        panel.addWidget(QLabel('每辆车容量:'))
        self.cap_table = QTableWidget(0, 2)
        self.cap_table.setHorizontalHeaderLabels(['机器人', '容量'])
        panel.addWidget(self.cap_table)

        panel.addWidget(QLabel('求解时限 (s):'))
        self.time_input = QLineEdit('10')
        panel.addWidget(self.time_input)

        panel.addWidget(QLabel('地图膨胀尺寸 inflate_size:'))
        self.inflate_input = QLineEdit('5')
        panel.addWidget(self.inflate_input)

        panel.addWidget(QLabel('客户需求 (左键添加，右键删除最近点):'))
        self.task_table = QTableWidget(0, 3)
        self.task_table.setHorizontalHeaderLabels(['客户索引', 'X', '需求'])
        panel.addWidget(self.task_table)

        btn_clear_tasks = QPushButton('清空任务点')
        btn_clear_tasks.clicked.connect(self.clear_tasks)
        panel.addWidget(btn_clear_tasks)

        btn_save = QPushButton('保存任务并关闭')
        btn_save.clicked.connect(self.save_yaml)
        panel.addWidget(btn_save)
        panel.addStretch()

        self._reset_canvas()

    def _load_initial_state(self):
        map_path_param = self.get_parameter('map_path').value
        default_map = os.path.join(self.base_dir, 'maps', 'manual_map_from_MAP.yaml')
        self.map_path = map_path_param or default_map
        self._load_map_image(self.map_path)

        if not self.map_derived_from_yaml:
            self.real_length_input.setText(str(self.get_parameter('real_length').value))
            self.origin_pixel_input.setText(self.get_parameter('origin_pixel').value)
            self.origin_world_input.setText(self.get_parameter('origin_world').value)
        self.robot_names_input.setText(self.get_parameter('robot_names').value)
        self.robot_topics_input.setText(self.get_parameter('robot_pose_topics').value)
        self.use_robot_starts_as_world_pose = bool(
            self.get_parameter('use_robot_starts_as_world_pose').value,
        )
        self.odometry_is_relative = bool(self.get_parameter('odometry_is_relative').value)
        self.odometry_origin_positions = _parse_json_list(
            self.get_parameter('odometry_origin_positions').value,
            'odometry_origin_positions',
        )
        self.odometry_origin_yaws_deg = [
            float(value)
            for value in _parse_json_list(
                self.get_parameter('odometry_origin_yaws_deg').value,
                'odometry_origin_yaws_deg',
            )
        ]

        pose_type = self.get_parameter('robot_pose_type').value
        index = self.pose_type_combo.findText(pose_type)
        if index >= 0:
            self.pose_type_combo.setCurrentIndex(index)

        self.time_input.setText(str(self.get_parameter('calculating_time').value))
        inflate_size = self.get_parameter('inflate_size').value
        if inflate_size <= 0:
            inflate_size = self.get_parameter('inflation_size').value
        if inflate_size <= 0:
            inflate_size = 5
        self.inflate_input.setText(str(inflate_size))

        vehicle_caps = _parse_json_list(self.get_parameter('vehicle_capacities').value, 'vehicle_capacities')
        self.vehicle_capacity_defaults = [int(cap) for cap in vehicle_caps]

        self.apply_map_params(show_message=False)
        self.apply_robot_config(show_message=False)

        customers = _parse_json_list(self.get_parameter('customers').value, 'customers')
        demands = _parse_json_list(self.get_parameter('demands').value, 'demands')
        self.customers = [[float(pt[0]), float(pt[1])] for pt in customers]
        self.demands = [int(value) for value in demands]
        self._sync_task_table()
        self._reset_canvas()

    def _load_map_image(self, map_path):
        self.map_img = None
        self.image_width = 0
        self.image_height = 0
        self.map_derived_from_yaml = False

        if not map_path:
            return
        if not os.path.exists(map_path):
            self.get_logger().warn(f'地图文件不存在: {map_path}')
            return

        try:
            map_config = load_map_config(map_path)
        except Exception as exc:
            self.get_logger().warn(f'无法读取地图文件: {map_path}: {exc}')
            return

        img = map_config['image']
        self.map_derived_from_yaml = bool(map_config['derived_from_yaml'])

        if img.ndim == 2:
            self.map_img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        else:
            self.map_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.image_height, self.image_width = img.shape[:2]

        if self.map_derived_from_yaml:
            self.real_length_input.setText(str(map_config['real_length']))
            self.origin_pixel_input.setText(json.dumps(map_config['origin_pixel'], ensure_ascii=False))
            self.origin_world_input.setText(json.dumps(map_config['origin_world'], ensure_ascii=False))

    def apply_map_params(self, show_message=True):
        try:
            self.real_length = float(self.real_length_input.text())
            self.origin_pixel = _parse_json_list(self.origin_pixel_input.text(), 'origin_pixel')
            self.origin_world = _parse_json_list(self.origin_world_input.text(), 'origin_world')

            if len(self.origin_pixel) != 2:
                raise ValueError('参考原点像素坐标必须包含两个元素')
            if len(self.origin_world) != 2:
                raise ValueError('参考原点世界坐标必须包含两个元素')
            if self.image_width <= 0 or self.image_height <= 0:
                raise ValueError('地图尺寸无效，请先提供有效地图')
            if self.real_length <= 0:
                raise ValueError('地图实际宽度必须大于 0')

            self.meters_per_pixel = self.real_length / self.image_width
            self.real_height = self.image_height * self.meters_per_pixel

            ul_x = -self.origin_pixel[0] * self.meters_per_pixel + self.origin_world[0]
            ul_y = self.origin_pixel[1] * self.meters_per_pixel + self.origin_world[1]
            lr_x = ul_x + self.image_width * self.meters_per_pixel
            lr_y = ul_y - self.image_height * self.meters_per_pixel
            self.map_extent = [ul_x, lr_x, lr_y, ul_y]

            self._reset_canvas()
            if show_message:
                QMessageBox.information(self, '成功', '地图参数已应用')
        except Exception as exc:
            QMessageBox.warning(self, '错误', f'地图参数错误: {exc}')

    def apply_robot_config(self, show_message=True):
        try:
            robot_names = _parse_json_list(self.robot_names_input.text(), 'robot_names')
            if not robot_names:
                raise ValueError('至少需要一个机器人名称')

            robot_names = [str(name).strip('/') for name in robot_names]
            if any(not name for name in robot_names):
                raise ValueError('机器人名称不能为空')

            pose_type = self.pose_type_combo.currentText()
            pose_topics = _parse_json_list(self.robot_topics_input.text(), 'robot_pose_topics')
            if pose_topics:
                if len(pose_topics) != len(robot_names):
                    raise ValueError('robot_pose_topics 数量必须与 robot_names 一致')
                pose_topics = [str(topic) for topic in pose_topics]
            else:
                if pose_type == 'pose_stamped':
                    default_suffix = 'pose'
                elif pose_type == 'odometry':
                    default_suffix = 'chassis/odom'
                else:
                    default_suffix = 'amcl_pose'
                pose_topics = [f'/{name}/{default_suffix}' for name in robot_names]
                self.robot_topics_input.setText(json.dumps(pose_topics, ensure_ascii=False))

            for subscription in self.pose_subscriptions:
                self.destroy_subscription(subscription)
            self.pose_subscriptions = []

            self.robot_names = robot_names
            self.robot_pose_topics = pose_topics
            initial_robot_starts = _parse_json_list(
                self.get_parameter('robot_starts').value,
                'robot_starts',
            )
            if self.odometry_is_relative and pose_type == 'odometry':
                if not self.odometry_origin_positions:
                    self.odometry_origin_positions = initial_robot_starts
                if not self.odometry_origin_yaws_deg:
                    self.odometry_origin_yaws_deg = [0.0] * len(robot_names)
                if len(self.odometry_origin_positions) != len(robot_names):
                    raise ValueError('odometry_origin_positions 数量必须与 robot_names 一致')
                if len(self.odometry_origin_yaws_deg) != len(robot_names):
                    raise ValueError('odometry_origin_yaws_deg 数量必须与 robot_names 一致')

            with self.robot_pose_lock:
                current_poses = dict(self.robot_poses)
                self.robot_poses = {}
                for index, name in enumerate(self.robot_names):
                    preserved = current_poses.get(name)
                    if preserved is not None:
                        self.robot_poses[name] = preserved
                    elif (
                        self.use_robot_starts_as_world_pose
                        and index < len(initial_robot_starts)
                    ):
                        self.robot_poses[name] = [
                            float(initial_robot_starts[index][0]),
                            float(initial_robot_starts[index][1]),
                        ]
                    else:
                        self.robot_poses[name] = None

            for robot_name, topic in zip(self.robot_names, self.robot_pose_topics):
                callback = self._make_pose_callback(robot_name)
                if pose_type == 'pose_stamped':
                    subscription = self.create_subscription(PoseStamped, topic, callback, 10)
                elif pose_type == 'odometry':
                    subscription = self.create_subscription(Odometry, topic, callback, 10)
                else:
                    subscription = self.create_subscription(
                        PoseWithCovarianceStamped,
                        topic,
                        callback,
                        10,
                    )
                self.pose_subscriptions.append(subscription)

            self._sync_capacity_table()
            self.robot_state_dirty = True
            self._refresh_robot_widgets()
            if show_message:
                QMessageBox.information(
                    self,
                    '成功',
                    '机器人配置已应用，正在等待实时位姿',
                )
        except Exception as exc:
            QMessageBox.warning(self, '错误', f'机器人配置错误: {exc}')

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
            position_xy = [pose.position.x, pose.position.y]
            if isinstance(msg, Odometry) and self.odometry_is_relative:
                robot_index = self.robot_names.index(robot_name)
                position_xy = _transform_relative_pose_to_world(
                    position_xy,
                    self.odometry_origin_positions[robot_index],
                    self.odometry_origin_yaws_deg[robot_index],
                )
            with self.robot_pose_lock:
                self.robot_poses[robot_name] = position_xy
            self.robot_state_dirty = True
        return callback

    def _sync_capacity_table(self):
        self.cap_table.setRowCount(len(self.robot_names))
        defaults = list(self.vehicle_capacity_defaults)

        for index, robot_name in enumerate(self.robot_names):
            name_item = QTableWidgetItem(robot_name)
            name_item.setFlags(name_item.flags() & ~Qt.ItemIsEditable)
            self.cap_table.setItem(index, 0, name_item)

            default_capacity = defaults[index] if index < len(defaults) else 0
            cap_item = QTableWidgetItem(str(default_capacity))
            cap_item.setFlags(cap_item.flags() | Qt.ItemIsEditable)
            self.cap_table.setItem(index, 1, cap_item)

    def _sync_task_table(self):
        self.task_table.setRowCount(len(self.customers))
        while len(self.demands) < len(self.customers):
            self.demands.append(0)

        for index, customer in enumerate(self.customers):
            index_item = QTableWidgetItem(str(index))
            index_item.setFlags(index_item.flags() & ~Qt.ItemIsEditable)
            self.task_table.setItem(index, 0, index_item)

            x_item = QTableWidgetItem(f'{customer[0]:.3f}, {customer[1]:.3f}')
            x_item.setFlags(x_item.flags() & ~Qt.ItemIsEditable)
            self.task_table.setItem(index, 1, x_item)

            demand_item = QTableWidgetItem(str(self.demands[index]))
            demand_item.setFlags(demand_item.flags() | Qt.ItemIsEditable)
            self.task_table.setItem(index, 2, demand_item)

    def _refresh_robot_widgets(self):
        if not self.robot_state_dirty:
            return
        self.robot_state_dirty = False

        with self.robot_pose_lock:
            robot_poses = dict(self.robot_poses)

        self.robot_table.setRowCount(len(self.robot_names))
        for row, (robot_name, topic) in enumerate(zip(self.robot_names, self.robot_pose_topics)):
            self.robot_table.setItem(row, 0, QTableWidgetItem(robot_name))
            self.robot_table.setItem(row, 1, QTableWidgetItem(topic))

            pose = robot_poses.get(robot_name)
            if pose is None:
                status = '等待中'
                x_text = '-'
                y_text = '-'
            else:
                status = '已收到'
                x_text = f'{pose[0]:.3f}'
                y_text = f'{pose[1]:.3f}'

            self.robot_table.setItem(row, 2, QTableWidgetItem(status))
            self.robot_table.setItem(row, 3, QTableWidgetItem(x_text))
            self.robot_table.setItem(row, 4, QTableWidgetItem(y_text))

        self._reset_canvas()

    def _reset_canvas(self):
        self.ax.clear()

        if self.map_img is not None and self.meters_per_pixel > 0:
            self.ax.imshow(
                self.map_img,
                origin='upper',
                extent=self.map_extent,
                aspect='auto',
            )
            self.ax.set_xlim(self.map_extent[0], self.map_extent[1])
            self.ax.set_ylim(self.map_extent[2], self.map_extent[3])
            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')
            self.ax.set_aspect(self.aspect_ratio)
            self.ax.plot(self.origin_world[0], self.origin_world[1], 'ko', markersize=4)
            self.ax.text(
                self.origin_world[0],
                self.origin_world[1],
                '参考原点',
                fontsize=9,
                ha='right',
                va='bottom',
            )
        elif self.map_img is not None:
            h, w = self.map_img.shape[:2]
            self.ax.imshow(self.map_img, origin='upper', extent=[0, w, h, 0])
            self.ax.set_xlim(0, w)
            self.ax.set_ylim(h, 0)
            self.ax.set_xlabel('X (pixels)')
            self.ax.set_ylabel('Y (pixels)')
        else:
            self.ax.set_xlim(0, 500)
            self.ax.set_ylim(0, 500)
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')

        with self.robot_pose_lock:
            robot_poses = dict(self.robot_poses)

        for robot_name in self.robot_names:
            pose = robot_poses.get(robot_name)
            if pose is None:
                continue
            self.ax.plot(pose[0], pose[1], 'bs', markersize=8)
            self.ax.text(
                pose[0],
                pose[1],
                robot_name,
                fontsize=8,
                ha='left',
                va='bottom',
                color='blue',
            )

        for index, (x, y) in enumerate(self.customers):
            self.ax.plot(x, y, 'r^', markersize=8)
            self.ax.text(x, y, f'C{index}', fontsize=8, ha='left', va='bottom', color='darkred')

        if self.meters_per_pixel <= 0:
            title = '请先设置地图参数'
        elif not self.robot_names:
            title = '请先配置机器人订阅'
        else:
            ready = sum(1 for name in self.robot_names if robot_poses.get(name) is not None)
            title = f'左键添加任务点，右键删除最近点 | 机器人位姿 {ready}/{len(self.robot_names)}'
        self.ax.set_title(title)
        self.ax.grid(True)
        self.canvas.draw()

    def on_click(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return
        if self.meters_per_pixel <= 0:
            QMessageBox.warning(self, '错误', '请先设置地图参数')
            return

        x = float(event.xdata)
        y = float(event.ydata)

        if event.button == 1:
            self.customers.append([x, y])
            self.demands.append(0)
        elif event.button == 3 and self.customers:
            nearest_index = min(
                range(len(self.customers)),
                key=lambda idx: (self.customers[idx][0] - x) ** 2 + (self.customers[idx][1] - y) ** 2,
            )
            self.customers.pop(nearest_index)
            self.demands.pop(nearest_index)
        else:
            return

        self._sync_task_table()
        self._reset_canvas()

    def clear_tasks(self):
        self.customers.clear()
        self.demands.clear()
        self._sync_task_table()
        self._reset_canvas()

    def save_yaml(self):
        if self.meters_per_pixel <= 0:
            QMessageBox.warning(self, '错误', '请先设置有效的地图参数')
            return
        if not self.robot_names:
            QMessageBox.warning(self, '错误', '请先配置机器人')
            return
        if not self.customers:
            QMessageBox.warning(self, '错误', '请至少设置一个任务点')
            return

        with self.robot_pose_lock:
            robot_starts = [self.robot_poses.get(name) for name in self.robot_names]
        if any(pose is None for pose in robot_starts):
            QMessageBox.warning(self, '错误', '仍有机器人尚未上报位姿，暂时不能保存')
            return

        try:
            self.demands = [
                int(self.task_table.item(row, 2).text())
                for row in range(self.task_table.rowCount())
            ]
            vehicle_caps = [
                int(self.cap_table.item(row, 1).text())
                for row in range(self.cap_table.rowCount())
            ]
            calc_time = int(self.time_input.text())
            inflate_size = int(self.inflate_input.text())
        except Exception:
            QMessageBox.warning(self, '错误', '容量、需求、求解时限和膨胀尺寸必须是整数')
            return

        follow_waypoints_actions = [
            f'/{robot_name}/follow_waypoints'
            for robot_name in self.robot_names
        ]

        shared_lines = [
            f'    robot_names: {_yaml_quote(json.dumps(self.robot_names, ensure_ascii=False))}',
            f'    robot_pose_topics: {_yaml_quote(json.dumps(self.robot_pose_topics, ensure_ascii=False))}',
            f'    robot_pose_type: {_yaml_quote(self.pose_type_combo.currentText())}',
            f'    robot_starts: {_yaml_quote(json.dumps(robot_starts, ensure_ascii=False))}',
            f'    use_robot_starts_as_world_pose: {"true" if self.use_robot_starts_as_world_pose else "false"}',
            f'    odometry_is_relative: {"true" if self.odometry_is_relative else "false"}',
            f'    odometry_origin_positions: {_yaml_quote(json.dumps(self.odometry_origin_positions, ensure_ascii=False))}',
            f'    odometry_origin_yaws_deg: {_yaml_quote(json.dumps(self.odometry_origin_yaws_deg, ensure_ascii=False))}',
            f'    customers: {_yaml_quote(json.dumps(self.customers, ensure_ascii=False))}',
            f'    demands: {_yaml_quote(json.dumps(self.demands, ensure_ascii=False))}',
            f'    num_vehicles: {len(self.robot_names)}',
            f'    vehicle_capacities: {_yaml_quote(json.dumps(vehicle_caps, ensure_ascii=False))}',
            f'    follow_waypoints_actions: {_yaml_quote(json.dumps(follow_waypoints_actions, ensure_ascii=False))}',
            '    auto_dispatch: true',
            f'    map_path: {_yaml_quote(self.map_path)}',
            f'    calculating_time: {calc_time}',
            f'    inflate_size: {inflate_size}',
            f'    real_length: {self.real_length}',
            f'    origin_pixel: {_yaml_quote(json.dumps(self.origin_pixel, ensure_ascii=False))}',
            f'    origin_world: {_yaml_quote(json.dumps(self.origin_world, ensure_ascii=False))}',
            f'    frame_id: {_yaml_quote(self.get_parameter("frame_id").value)}',
            f'    pose_wait_timeout: {float(self.get_parameter("pose_wait_timeout").value)}',
            f'    nav2_wait_timeout: {float(self.get_parameter("nav2_wait_timeout").value)}',
            f'    visualize: {"true" if self.get_parameter("visualize").value else "false"}',
        ]
        yaml_lines = [
            'params_generator:',
            '  ros__parameters:',
            *shared_lines,
            'cvrp_planner_node:',
            '  ros__parameters:',
            *shared_lines,
        ]

        file_name = os.path.join(self.base_dir, 'params', 'params.yaml')
        try:
            os.makedirs(os.path.dirname(file_name), exist_ok=True)
            with open(file_name, 'w', encoding='utf-8') as file_obj:
                file_obj.write('\n'.join(yaml_lines))
        except Exception as exc:
            QMessageBox.warning(self, '错误', f'YAML 保存失败: {exc}')
            return

        params = [
            Parameter('robot_names', Parameter.Type.STRING, json.dumps(self.robot_names, ensure_ascii=False)),
            Parameter('robot_pose_topics', Parameter.Type.STRING, json.dumps(self.robot_pose_topics, ensure_ascii=False)),
            Parameter('robot_pose_type', Parameter.Type.STRING, self.pose_type_combo.currentText()),
            Parameter('robot_starts', Parameter.Type.STRING, json.dumps(robot_starts, ensure_ascii=False)),
            Parameter(
                'use_robot_starts_as_world_pose',
                Parameter.Type.BOOL,
                self.use_robot_starts_as_world_pose,
            ),
            Parameter('odometry_is_relative', Parameter.Type.BOOL, self.odometry_is_relative),
            Parameter(
                'odometry_origin_positions',
                Parameter.Type.STRING,
                json.dumps(self.odometry_origin_positions, ensure_ascii=False),
            ),
            Parameter(
                'odometry_origin_yaws_deg',
                Parameter.Type.STRING,
                json.dumps(self.odometry_origin_yaws_deg, ensure_ascii=False),
            ),
            Parameter('customers', Parameter.Type.STRING, json.dumps(self.customers, ensure_ascii=False)),
            Parameter('demands', Parameter.Type.STRING, json.dumps(self.demands, ensure_ascii=False)),
            Parameter('num_vehicles', Parameter.Type.INTEGER, len(self.robot_names)),
            Parameter('vehicle_capacities', Parameter.Type.STRING, json.dumps(vehicle_caps, ensure_ascii=False)),
            Parameter(
                'follow_waypoints_actions',
                Parameter.Type.STRING,
                json.dumps(follow_waypoints_actions, ensure_ascii=False),
            ),
            Parameter('auto_dispatch', Parameter.Type.BOOL, True),
            Parameter('map_path', Parameter.Type.STRING, self.map_path),
            Parameter('calculating_time', Parameter.Type.INTEGER, calc_time),
            Parameter('inflate_size', Parameter.Type.INTEGER, inflate_size),
            Parameter('real_length', Parameter.Type.DOUBLE, self.real_length),
            Parameter('origin_pixel', Parameter.Type.STRING, json.dumps(self.origin_pixel, ensure_ascii=False)),
            Parameter('origin_world', Parameter.Type.STRING, json.dumps(self.origin_world, ensure_ascii=False)),
            Parameter('frame_id', Parameter.Type.STRING, self.get_parameter('frame_id').value),
            Parameter('pose_wait_timeout', Parameter.Type.DOUBLE, float(self.get_parameter('pose_wait_timeout').value)),
            Parameter('nav2_wait_timeout', Parameter.Type.DOUBLE, float(self.get_parameter('nav2_wait_timeout').value)),
            Parameter('visualize', Parameter.Type.BOOL, bool(self.get_parameter('visualize').value)),
        ]
        self.set_parameters(params)

        QMessageBox.information(
            self,
            '成功',
            f'任务参数已保存到 {file_name}\n关闭窗口后会继续启动规划节点。',
        )
        self.close()
        QApplication.instance().quit()


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    node = CVRPPlannerGUINode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    node.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
