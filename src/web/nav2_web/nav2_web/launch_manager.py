"""Allowlisted ROS 2 launch process manager for the Nav2 Web console."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import os
import re
import signal
import subprocess
import threading
import time
from typing import Callable, Optional


_ANSI_ESCAPE = re.compile(
    r'\x1b(?:\[[0-?]*[ -/]*[@-~]|\][^\x07]*(?:\x07|\x1b\\))'
)
_LIVOX_FATAL_OUTPUT = (
    'bind failed',
    'failed to init livox lidar sdk',
    'init lds lidar fail',
)

# T_imu_body for a Mid-360S mounted 0.15 m forward and 0.05 m above trunk
# (the dog IMU origin), with its x/front side tilted down 20 degrees.
_BODY_X_IN_IMU_M = -0.1348528860
_BODY_Y_IN_IMU_M = -0.02329
_BODY_Z_IN_IMU_M = -0.0541676525
_BODY_PITCH_IN_IMU_RAD = -0.3490658504


class LaunchManagerError(ValueError):
    """A launch-control request was rejected before changing process state."""


@dataclass(frozen=True)
class LaunchArgument:
    """One typed launch argument that a trusted App client may override."""

    name: str
    label: str
    kind: str
    default: object
    group: str
    description: str = ''
    minimum: Optional[float] = None
    maximum: Optional[float] = None
    choices: tuple[tuple[str, str], ...] = ()

    def public_dict(self) -> dict:
        """Describe the field without exposing arbitrary command execution."""
        return {
            'name': self.name,
            'label': self.label,
            'kind': self.kind,
            'default': self.default,
            'group': self.group,
            'description': self.description,
            'minimum': self.minimum,
            'maximum': self.maximum,
            'choices': [
                {'value': value, 'label': label}
                for value, label in self.choices
            ],
        }

    def launch_value(self, raw_value: object) -> str:
        """Validate a Web value and serialize it as one ROS launch argument."""
        if self.kind == 'bool':
            if isinstance(raw_value, bool):
                return 'true' if raw_value else 'false'
            normalized = str(raw_value).strip().lower()
            if normalized not in ('true', 'false', '1', '0', 'yes', 'no'):
                raise LaunchManagerError(f'{self.label} 必须是开或关')
            return 'true' if normalized in ('true', '1', 'yes') else 'false'

        if self.kind in ('float', 'int'):
            try:
                number = float(raw_value)
            except (TypeError, ValueError) as error:
                raise LaunchManagerError(f'{self.label} 必须是数字') from error
            if not math.isfinite(number):
                raise LaunchManagerError(f'{self.label} 必须是有限数值')
            if self.minimum is not None and number < self.minimum:
                raise LaunchManagerError(
                    f'{self.label} 不能小于 {self.minimum:g}')
            if self.maximum is not None and number > self.maximum:
                raise LaunchManagerError(
                    f'{self.label} 不能大于 {self.maximum:g}')
            if self.kind == 'int':
                if not number.is_integer():
                    raise LaunchManagerError(f'{self.label} 必须是整数')
                return str(int(number))
            return format(number, '.12g')

        value = str(raw_value).strip()
        if self.kind == 'choice':
            allowed = {choice[0] for choice in self.choices}
            if value not in allowed:
                raise LaunchManagerError(f'{self.label} 不是允许的选项')
            return value
        if self.kind == 'frame':
            if re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]{0,95}', value) is None:
                raise LaunchManagerError(
                    f'{self.label} 必须是合法 TF frame，且不能以 / 开头')
            return value
        if self.kind == 'topic':
            if re.fullmatch(r'/[A-Za-z0-9_~/]+', value) is None:
                raise LaunchManagerError(f'{self.label} 必须是绝对 ROS Topic')
            return value
        raise LaunchManagerError(f'不支持的 App 参数类型: {self.kind}')


@dataclass(frozen=True)
class LaunchProfile:
    """A launch entry that the Web client is explicitly allowed to start."""

    profile_id: str
    label: str
    description: str
    package: str
    launch_file: str
    arguments: tuple[str, ...] = ()
    requires_map: bool = False
    requires_pbstream: bool = False
    stage: str = 'navigation'
    dangerous: bool = False
    navigation_mode: Optional[int] = None
    control_enabled: bool = False
    available: bool = True
    unavailable_reason: str = ''
    configurable_arguments: tuple[LaunchArgument, ...] = ()
    map_role: str = ''
    default_map_name: str = ''
    save_before_stop: bool = False

    def public_dict(self) -> dict:
        """Return the fields that are safe to expose to Web clients."""
        return {
            'id': self.profile_id,
            'label': self.label,
            'description': self.description,
            'requires_map': self.requires_map,
            'requires_pbstream': self.requires_pbstream,
            'stage': self.stage,
            'dangerous': self.dangerous,
            'navigation_mode': self.navigation_mode,
            'control_enabled': self.control_enabled,
            'available': self.available,
            'unavailable_reason': self.unavailable_reason,
            'map_role': self.map_role,
            'default_map_name': self.default_map_name,
            'save_before_stop': self.save_before_stop,
            'parameters': [
                argument.public_dict()
                for argument in self.configurable_arguments
            ],
        }


def dog_launch_profiles() -> tuple[LaunchProfile, ...]:
    """Return the fixed launch allowlist for this robot workspace."""
    package = 'dog_cartographer_nav2_bringup'
    return (
        LaunchProfile(
            profile_id='cartographer_mapping',
            label='Cartographer 建图',
            description='纯激光扫描建图，Cartographer 负责 map/odom TF',
            package=package,
            launch_file='cartographer_mapping.launch.py',
            arguments=('start_web:=False', 'start_rf2o_prior:=False'),
        ),
        LaunchProfile(
            profile_id='cartographer_rf2o_mapping',
            label='Cartographer + RF2O 建图',
            description='RF2O 作为里程计先验，Cartographer 仍管理 TF',
            package=package,
            launch_file='cartographer_mapping.launch.py',
            arguments=(
                'start_web:=False',
                'start_rf2o_prior:=True',
                'cartographer_configuration_basename:='
                'ydlidar_cartographer_2d_rf2o.lua',
                'rf2o_odom_topic:=/odom_rf2o',
            ),
        ),
        LaunchProfile(
            profile_id='nav2_mppi_amcl',
            label='AMCL + MPPI 导航',
            description='加载 YAML 地图，使用 AMCL 定位和 MPPI 控制',
            package=package,
            launch_file='nav2_mppi_navigation.launch.py',
            arguments=('start_web:=False',),
            requires_map=True,
        ),
        LaunchProfile(
            profile_id='cartographer_mppi_navigation',
            label='图 SLAM + MPPI 导航',
            description='加载 YAML 和同名 PBSTREAM，使用冻结图定位',
            package=package,
            launch_file='cartographer_mppi_navigation.launch.py',
            arguments=('start_web:=False',),
            requires_map=True,
            requires_pbstream=True,
        ),
    )


def _field(
    name: str,
    label: str,
    kind: str,
    default: object,
    group: str,
    description: str = '',
    minimum: Optional[float] = None,
    maximum: Optional[float] = None,
    choices: tuple[tuple[str, str], ...] = (),
) -> LaunchArgument:
    return LaunchArgument(
        name=name, label=label, kind=kind, default=default, group=group,
        description=description, minimum=minimum, maximum=maximum,
        choices=choices,
    )


def _physical_arguments(
    *,
    driver: bool = True,
    fastlio: bool = True,
    planner: bool = True,
    localization: bool = False,
    pct: bool = False,
) -> tuple[LaunchArgument, ...]:
    """App-editable physical parameters shared by allowlisted workflows."""
    component_fields = [
        _field('start_livox_driver', '启动 Mid-360S 驱动', 'bool', driver,
               '组件组合', '已有外部驱动时关闭，避免重复占用雷达。'),
        _field('start_rviz', '机器人端启动 RViz', 'bool', False,
               '组件组合', '通常使用 App 时关闭。'),
    ]
    if localization:
        component_fields.extend([
            _field('start_localization', '历史地图重定位', 'bool', True,
                   '组件组合', '包含 ICP、Localization FAST-LIO 与全局适配器。'),
            _field('start_pct', 'PCT 全局规划', 'bool', pct,
                   '组件组合', '接收单目标或多途经点并发布 /initial_path。'),
            _field('start_scanplanner', 'SCAN-Planner 局部规划', 'bool', planner,
                   '组件组合', '历史地图流程固定使用 SCAN 模式 3。'),
        ])
    else:
        component_fields.extend([
            _field('start_fastlio', 'FAST-LIO 定位', 'bool', fastlio,
                   '组件组合', '已有外部 FAST-LIO 时可以关闭。'),
            _field('start_scanplanner', 'SCAN-Planner', 'bool', planner,
                   '组件组合', '关闭后只保留雷达/定位诊断。'),
        ])

    physical_fields = [
        _field('fastlio_imu_frame', 'FAST-LIO IMU frame', 'frame',
               'fastlio_imu' if localization else 'body', '机身与 TF',
               '与真实 URDF frame 重名时使用独立名称。'),
        _field('body_frame', '机身根坐标系', 'choice', 'trunk', '机身与 TF',
               '当前机器狗的 trunk 原点位于机身 IMU。', choices=(
                   ('trunk', 'trunk'), ('base', 'base'))),
        _field('lidar_frame', '雷达坐标系', 'frame', 'livox_frame', '机身与 TF'),
        _field('publish_lidar_tf', '由适配器发布机身→雷达 TF', 'bool', True,
               '机身与 TF', '真实 URDF 已发布同一 TF 时关闭。'),
        _field('body_in_imu_x', '机身在 IMU 中 X', 'float',
               _BODY_X_IN_IMU_M, '安装外参',
               '雷达在 trunk 前方15 cm、上方5 cm且下倾20°，并合并 Mid360 内部外参后的 T_imu_body X。',
               -5.0, 5.0),
        _field('body_in_imu_y', '机身在 IMU 中 Y', 'float',
               _BODY_Y_IN_IMU_M, '安装外参',
               '最终 trunk→LiDAR 左右居中；此值包含 Mid360 内部 IMU→LiDAR Y 外参。',
               -5.0, 5.0),
        _field('body_in_imu_z', '机身在 IMU 中 Z', 'float',
               _BODY_Z_IN_IMU_M, '安装外参',
               '雷达在 trunk 前方15 cm、上方5 cm且下倾20°，并合并 Mid360 内部外参后的 T_imu_body Z。',
               -5.0, 5.0),
        _field('body_rpy_in_imu_x', '机身在 IMU 中 Roll', 'float', 0.0,
               '安装外参', '单位 rad。', -3.141593, 3.141593),
        _field('body_rpy_in_imu_y', '机身在 IMU 中 Pitch', 'float',
               _BODY_PITCH_IN_IMU_RAD, '安装外参',
               '当前默认：雷达前端向下20°，T_imu_body=-0.349066 rad；前端向上则改为正值。',
               -3.141593, 3.141593),
        _field('body_rpy_in_imu_z', '机身在 IMU 中 Yaw', 'float', 0.0,
               '安装外参', '单位 rad。', -3.141593, 3.141593),
        _field('lidar_in_imu_x', 'LiDAR 在 IMU 中 X', 'float', -0.011,
               'MID360 内部外参', 'T_imu_lidar；不要用它填写整机安装倾角。', -1.0, 1.0),
        _field('lidar_in_imu_y', 'LiDAR 在 IMU 中 Y', 'float', -0.02329,
               'MID360 内部外参', 'T_imu_lidar；保持与稳定 FAST-LIO 配置一致。', -1.0, 1.0),
        _field('lidar_in_imu_z', 'LiDAR 在 IMU 中 Z', 'float', 0.04412,
               'MID360 内部外参', 'T_imu_lidar；保持与稳定 FAST-LIO 配置一致。', -1.0, 1.0),
        _field('lidar_rpy_in_imu_x', 'LiDAR/IMU Roll', 'float', 0.0,
               'MID360 内部外参', '单位 rad。', -3.141593, 3.141593),
        _field('lidar_rpy_in_imu_y', 'LiDAR/IMU Pitch', 'float', 0.0,
               'MID360 内部外参', '单位 rad；不是雷达相对机身的20°安装倾角。',
               -3.141593, 3.141593),
        _field('lidar_rpy_in_imu_z', 'LiDAR/IMU Yaw', 'float', 0.0,
               'MID360 内部外参', '单位 rad。', -3.141593, 3.141593),
        _field('cmd_vel_topic', '机器狗速度 Topic', 'topic', '/cmd_vel',
               '控制输出', '仅实机控制模式真正发布。'),
    ]
    if not localization:
        physical_fields.insert(3, _field(
            'publish_base_tf', '由适配器发布世界→机身 TF', 'bool', True,
            '机身与 TF', '只有其他节点拥有完全相同 child TF 时才关闭。'))
    if localization:
        physical_fields.insert(3, _field(
            'body_height', '机身中心离地高度', 'float', 0.4, '机身与 TF',
            '单位 m；同时用于 PCT 和交互初始点。', 0.05, 2.0))
    return tuple(component_fields + physical_fields)


def _relocalization_arguments() -> tuple[LaunchArgument, ...]:
    return (
        _field('initial_x', '初始 X', 'float', 0.0, 'ICP 初始位姿',
               '机器人机身在历史 map 中的粗略位置。', -10000.0, 10000.0),
        _field('initial_y', '初始 Y', 'float', 0.0, 'ICP 初始位姿',
               '机器人机身在历史 map 中的粗略位置。', -10000.0, 10000.0),
        _field('initial_z', '初始机身 Z', 'float', 0.4, 'ICP 初始位姿',
               '这是机身中心高度，不是地面高度。', -1000.0, 1000.0),
        _field('initial_yaw', '初始 Yaw', 'float', 0.0, 'ICP 初始位姿',
               '单位 rad。', -3.141593, 3.141593),
        _field('icp_max_correspondence_distance', 'ICP 最大对应距离', 'float',
               1.0, 'ICP 参数', '单位 m；增大不等于一定能全局重定位。', 0.05, 10.0),
        _field('icp_fitness_threshold', 'ICP 接受分数', 'float', 0.30,
               'ICP 参数', '越小越严格。', 0.0001, 10.0),
        _field('icp_required_convergences', 'ICP 连续成功帧数', 'int', 5,
               'ICP 参数', '连续达到阈值后才固定 map→odom。', 1, 100),
    )


def scanplanner_launch_profiles(
    keypoints_file: str = '',
    reference_path_file: str = '',
    map_file: str = '',
    tomogram_file: str = '',
) -> tuple[LaunchProfile, ...]:
    """Return physical Mid-360S/FAST-LIO/SCAN-Planner launch modes."""
    common = (
        'start_rviz:=false',
        'start_mobile_3d:=false',
        'publish_robot_state:=false',
    )
    package = 'scan_planner'
    launch_file = 'real_fastlio.launch.py'
    keypoints_file = os.path.realpath(os.path.expanduser(keypoints_file)) \
        if keypoints_file else ''
    reference_path_file = os.path.realpath(
        os.path.expanduser(reference_path_file)
    ) if reference_path_file else ''
    map_file = os.path.realpath(os.path.expanduser(map_file)) \
        if map_file else ''
    tomogram_file = os.path.realpath(os.path.expanduser(tomogram_file)) \
        if tomogram_file else ''
    keypoints_ready = bool(
        keypoints_file and os.path.isfile(keypoints_file)
    )
    reference_ready = bool(
        reference_path_file and os.path.isfile(reference_path_file)
    )
    map_stem = os.path.splitext(os.path.basename(map_file))[0] \
        if map_file else ''
    tomogram_stem = os.path.splitext(os.path.basename(tomogram_file))[0] \
        if tomogram_file else ''
    default_bundle = map_stem if map_stem == tomogram_stem else ''
    # Keep the phone surface small while exposing the options that are useful
    # on every run. Physical calibration and ICP tuning stay in launch/YAML.
    rviz_parameter = _field(
        'start_rviz', '电脑同时打开 RViz', 'bool', False, '电脑显示',
        '由手机启动当前流程时，同时在电脑图形桌面打开 RViz。')
    body_height_parameter = _field(
        'body_height', '地面到 trunk 高度', 'float', 0.40, 'PCT 高度',
        '单位 m；用于 PCT 路径、粗定位和机身中心高度。', 0.05, 2.0)
    driver_parameters = (rviz_parameter,)
    fastlio_parameters = (rviz_parameter,)
    live_parameters = (rviz_parameter,)
    historical_parameters = (rviz_parameter, body_height_parameter)

    base_profiles = (
        LaunchProfile(
            profile_id='mid360s_terminal',
            label='Mid-360S 雷达终端',
            description='启动 Mid-360S 专用驱动，检查 /livox/lidar 与 /livox/imu',
            package=package,
            launch_file=launch_file,
            arguments=common + (
                'start_livox_driver:=true',
                'start_fastlio:=false',
                'start_scanplanner:=false',
                'enable_control:=false',
            ),
            stage='lidar',
            configurable_arguments=driver_parameters,
        ),
        LaunchProfile(
            profile_id='fastlio_terminal',
            label='FAST-LIO 定位终端',
            description='启动 Mid-360S 与 FAST-LIO，只验证定位和注册点云',
            package=package,
            launch_file=launch_file,
            arguments=common + (
                'start_livox_driver:=true',
                'start_fastlio:=true',
                'start_scanplanner:=false',
                'enable_control:=false',
            ),
            stage='localization',
            configurable_arguments=fastlio_parameters,
        ),
    )

    mode_descriptions = {
        1: '手动目标：接收 /move_base_simple/goal 后规划',
        2: '预设航点：按 YAML 中的 fsm.waypoints 自动依次运行',
        3: '参考路径：订阅 /initial_path 并进行局部避障',
    }
    planner_profiles = []
    for control_enabled in (False, True):
        suffix = 'control' if control_enabled else 'preview'
        label_suffix = '实机控制' if control_enabled else '安全预览'
        for navigation_mode in (1, 2, 3):
            extra_arguments = [
                'start_livox_driver:=true',
                'start_fastlio:=true',
                'start_scanplanner:=true',
                f'enable_control:={str(control_enabled).lower()}',
                f'navi_mode:={navigation_mode}',
            ]
            available = True
            unavailable_reason = ''
            if navigation_mode == 2:
                available = keypoints_ready
                if keypoints_ready:
                    extra_arguments.append(
                        f'keypoints_file:={keypoints_file}'
                    )
                else:
                    unavailable_reason = (
                        '机器人端未配置模式 2 的 keypoints_file'
                    )
            elif navigation_mode == 3 and reference_ready:
                extra_arguments.append(
                    f'reference_path_file:={reference_path_file}'
                )

            description = mode_descriptions[navigation_mode]
            if control_enabled:
                description += '；发布 /cmd_vel，线速度与角速度均不超过 0.75'
            else:
                description += '；只规划和显示，不发布 /cmd_vel，机器狗不会移动'
            if navigation_mode == 3 and not reference_ready:
                description += '；等待外部节点发布路径'
            planner_profiles.append(LaunchProfile(
                profile_id=f'scanplanner_mode{navigation_mode}_{suffix}',
                label=f'模式 {navigation_mode} · {label_suffix}',
                description=description,
                package=package,
                launch_file=launch_file,
                arguments=common + tuple(extra_arguments),
                stage='control' if control_enabled else 'planning',
                dangerous=control_enabled,
                navigation_mode=navigation_mode,
                control_enabled=control_enabled,
                available=available,
                unavailable_reason=unavailable_reason,
                configurable_arguments=live_parameters,
            ))
    workflow_profiles = (
        LaunchProfile(
            profile_id='fastlio_global_mapping',
            label='FAST-LIO 全局 PCD 建图',
            description='MID360S 建图；停止时先自动保存 PCD，成功后才结束流程',
            package=package,
            launch_file='build_global_map.launch.py',
            arguments=(
                'start_livox_driver:=true', 'start_rviz:=false',
                f'map_file:={map_file}',
            ),
            stage='mapping',
            map_role='pcd_output',
            default_map_name=map_stem,
            save_before_stop=True,
            configurable_arguments=(rviz_parameter,),
        ),
        LaunchProfile(
            profile_id='pct_tomogram_build',
            label='PCD → PCT 全局图',
            description='离线生成 PCT tomogram；看到 Tomogram exported 后停止即可',
            package=package,
            launch_file='build_pct_tomogram.launch.py',
            arguments=(
                f'map_file:={map_file}', f'tomogram_file:={tomogram_file}',
            ),
            stage='mapping',
            map_role='tomogram',
            default_map_name=map_stem,
            configurable_arguments=(
                rviz_parameter,
                _field('ground_height', '最低可走地面 Z', 'float', 0.0,
                       'PCT 地图', '历史 map 坐标中的地面高度。', -1000.0, 1000.0),
                _field('slice_height', 'PCT 垂直分层间隔', 'float', 0.50,
                       'PCT 地图', '单位 m；普通楼梯和多层地图保持 0.50。', 0.10, 5.0),
            ),
        ),
        LaunchProfile(
            profile_id='pct_offline_demo',
            label='PCT 示例地图 · App 两点测试',
            description=(
                '固定虚拟起点 + 历史 PCD + PCT；只显示全局轨迹，'
                '不启动雷达、SCAN 或速度控制'),
            package=package,
            launch_file='pct_offline_demo.launch.py',
            arguments=(
                f'map_file:={map_file}', f'tomogram_file:={tomogram_file}',
            ),
            stage='planning', navigation_mode=3,
            map_role='bundle',
            default_map_name=default_bundle,
            configurable_arguments=(rviz_parameter, body_height_parameter),
        ),
        LaunchProfile(
            profile_id='pct_scanplanner_mode3_preview',
            label='历史地图模式 3 · 安全预览',
            description='PCD 重定位 + PCT 全局路径 + SCAN 局部避障，不输出速度',
            package=package,
            launch_file='prior_map_navigation.launch.py',
            arguments=(
                'start_livox_driver:=true', 'start_rviz:=false',
                'start_mobile_3d:=false', 'enable_control:=false',
                f'map_file:={map_file}', f'tomogram_file:={tomogram_file}',
            ),
            stage='planning', navigation_mode=3,
            map_role='bundle',
            default_map_name=default_bundle,
            configurable_arguments=historical_parameters,
        ),
        LaunchProfile(
            profile_id='pct_scanplanner_mode3_control',
            label='历史地图模式 3 · 实机控制',
            description='PCD 重定位 + PCT + SCAN 闭环控制，确认预览正确后使用',
            package=package,
            launch_file='prior_map_navigation.launch.py',
            arguments=(
                'start_livox_driver:=true', 'start_rviz:=false',
                'start_mobile_3d:=false', 'enable_control:=true',
                f'map_file:={map_file}', f'tomogram_file:={tomogram_file}',
            ),
            stage='control', dangerous=True, navigation_mode=3,
            control_enabled=True,
            map_role='bundle',
            default_map_name=default_bundle,
            configurable_arguments=historical_parameters,
        ),
    )
    return base_profiles + tuple(planner_profiles) + workflow_profiles


def launch_profiles_for(
    profile_set: str,
    keypoints_file: str = '',
    reference_path_file: str = '',
    map_file: str = '',
    tomogram_file: str = '',
) -> tuple[LaunchProfile, ...]:
    """Select one built-in allowlist; unknown values stay on the Nav2 set."""
    if str(profile_set).strip().lower() == 'scanplanner':
        return scanplanner_launch_profiles(
            keypoints_file=keypoints_file,
            reference_path_file=reference_path_file,
            map_file=map_file,
            tomogram_file=tomogram_file,
        )
    return dog_launch_profiles()


class LaunchManager:
    """Start one allowlisted ROS launch at a time and retain bounded logs."""

    def __init__(
        self,
        enabled: bool,
        map_directory: str,
        event_callback: Optional[Callable[[dict], None]] = None,
        profiles: Optional[tuple[LaunchProfile, ...]] = None,
        command_factory: Optional[
            Callable[[LaunchProfile, list[str]], list[str]]
        ] = None,
        max_log_lines: int = 1200,
    ):
        """Create a manager without starting any subprocesses."""
        self.enabled = bool(enabled)
        self.map_directory = os.path.realpath(os.path.abspath(
            os.path.expanduser(map_directory)
        ))
        profile_values = profiles or dog_launch_profiles()
        self._profiles = {
            profile.profile_id: profile for profile in profile_values
        }
        self._event_callback = event_callback
        self._command_factory = command_factory or self._ros2_command
        self._lock = threading.RLock()
        self._logs: deque[dict] = deque(maxlen=max(100, max_log_lines))
        self._log_sequence = 0
        self._process: Optional[subprocess.Popen] = None
        self._active_profile: Optional[LaunchProfile] = None
        self._active_map_name: Optional[str] = None
        self._active_parameters: dict[str, object] = {}
        self._state = 'idle'
        self._exit_code: Optional[int] = None
        self._started_at: Optional[float] = None
        self._command: list[str] = []
        self._shutdown = False

    @staticmethod
    def _ros2_command(
        profile: LaunchProfile,
        arguments: list[str],
    ) -> list[str]:
        override_names = {
            argument.split(':=', 1)[0]
            for argument in arguments if ':=' in argument
        }
        fixed_arguments = [
            argument for argument in profile.arguments
            if argument.split(':=', 1)[0] not in override_names
        ]
        return [
            'ros2', 'launch', profile.package, profile.launch_file,
            *fixed_arguments, *arguments,
        ]

    def _emit(self, message: dict):
        if self._event_callback is None:
            return
        try:
            self._event_callback(message)
        except Exception:
            # A disconnected Web client must never affect the ROS process.
            pass

    def _append_log(self, line: str, level: str = 'output'):
        clean_line = _ANSI_ESCAPE.sub('', line).replace('\x00', '').rstrip()
        if not clean_line:
            return
        if len(clean_line) > 4000:
            clean_line = f'{clean_line[:4000]}…'
        with self._lock:
            self._log_sequence += 1
            entry = {
                'seq': self._log_sequence,
                'time': time.strftime('%H:%M:%S'),
                'level': level,
                'line': clean_line,
            }
            self._logs.append(entry)
        self._emit({'type': 'launch_log', 'entry': entry})

    def _map_catalog(self) -> list[dict]:
        try:
            names = sorted(os.listdir(self.map_directory))
        except OSError:
            return []
        maps = []
        scanplanner_maps: dict[str, dict] = {}
        for name in names:
            if name != os.path.basename(name):
                continue
            candidate = os.path.realpath(os.path.join(
                self.map_directory, name))
            if (not os.path.isfile(candidate) or os.path.commonpath(
                    (self.map_directory, candidate)) != self.map_directory):
                continue
            if name.endswith('.yaml'):
                stem = name[:-5]
                maps.append({
                    'kind': 'nav2',
                    'name': name,
                    'label': stem,
                    'has_pbstream': os.path.isfile(os.path.join(
                        self.map_directory, f'{stem}.pbstream'
                    )),
                })
                continue
            suffix = next((value for value in ('.pcd', '.pickle')
                           if name.endswith(value)), '')
            if not suffix:
                continue
            stem = name[:-len(suffix)]
            entry = scanplanner_maps.setdefault(stem, {
                'kind': 'scanplanner',
                'name': stem,
                'label': stem,
                'has_pcd': False,
                'has_tomogram': False,
            })
            entry['has_pcd' if suffix == '.pcd' else 'has_tomogram'] = True
        maps.extend(scanplanner_maps[name] for name in sorted(scanplanner_maps))
        return maps

    def snapshot(self) -> dict:
        """Return catalog, current process state, maps, and retained logs."""
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            active = None
            if self._active_profile is not None:
                active = {
                    'profile_id': self._active_profile.profile_id,
                    'label': self._active_profile.label,
                    'state': self._state,
                    'running': running,
                    'pid': process.pid if running else None,
                    'map_name': self._active_map_name,
                    'save_before_stop': self._active_profile.save_before_stop,
                    'parameters': dict(self._active_parameters),
                    'started_at': self._started_at,
                    'exit_code': self._exit_code,
                }
            logs = list(self._logs)
        return {
            'type': 'launch_status',
            'enabled': self.enabled,
            'profiles': [
                profile.public_dict() for profile in self._profiles.values()
            ],
            'maps': self._map_catalog(),
            'active': active,
            'logs': logs,
            'map_directory': self.map_directory,
        }

    def _notify_status(self):
        self._emit(self.snapshot())

    def _resolve_map_arguments(
        self,
        profile: LaunchProfile,
        map_name: Optional[str],
    ) -> tuple[list[str], Optional[str]]:
        if profile.map_role:
            return self._resolve_scanplanner_map_arguments(profile, map_name)
        if not profile.requires_map:
            return [], None
        if not map_name or map_name != os.path.basename(map_name):
            raise LaunchManagerError('请从 maps 目录选择一个 YAML 地图')
        if not map_name.endswith('.yaml'):
            raise LaunchManagerError('导航地图必须是 .yaml 文件')
        map_path = os.path.realpath(os.path.join(self.map_directory, map_name))
        if os.path.commonpath(
            (self.map_directory, map_path)
        ) != self.map_directory:
            raise LaunchManagerError('地图路径超出允许的 maps 目录')
        if not os.path.isfile(map_path):
            raise LaunchManagerError(f'地图不存在: {map_name}')
        arguments = [f'map:={map_path}']
        if profile.requires_pbstream:
            pbstream_path = os.path.splitext(map_path)[0] + '.pbstream'
            if not os.path.isfile(pbstream_path):
                raise LaunchManagerError(
                    f'图 SLAM 导航缺少同名文件: '
                    f'{os.path.basename(pbstream_path)}'
                )
            arguments.append(f'pbstream:={pbstream_path}')
        return arguments, map_name

    @staticmethod
    def _normalize_scanplanner_map_name(value: Optional[str]) -> str:
        name = str(value or '').strip()
        for suffix in ('.pickle', '.pcd'):
            if name.endswith(suffix):
                name = name[:-len(suffix)]
                break
        if (not name or len(name) > 80 or name in ('.', '..') or
                re.fullmatch(r'[\w][\w.-]*', name) is None):
            raise LaunchManagerError(
                '地图名只能使用中文、字母、数字、下划线、点和短横线')
        return name

    def _scanplanner_map_path(self, name: str, suffix: str) -> str:
        path = os.path.realpath(os.path.join(
            self.map_directory, f'{name}{suffix}'))
        if os.path.commonpath((self.map_directory, path)) != self.map_directory:
            raise LaunchManagerError('地图路径超出允许的地图库目录')
        return path

    def _resolve_scanplanner_map_arguments(
        self,
        profile: LaunchProfile,
        map_name: Optional[str],
    ) -> tuple[list[str], str]:
        name = self._normalize_scanplanner_map_name(
            map_name or profile.default_map_name)
        pcd_path = self._scanplanner_map_path(name, '.pcd')
        tomogram_path = self._scanplanner_map_path(name, '.pickle')
        role = profile.map_role

        if role == 'pcd_output':
            os.makedirs(self.map_directory, exist_ok=True)
            if os.path.exists(pcd_path) or os.path.exists(tomogram_path):
                raise LaunchManagerError(
                    f'地图 {name} 已存在；为避免覆盖，请输入新的地图名')
            return [f'map_file:={pcd_path}'], name
        if role not in ('tomogram', 'bundle'):
            raise LaunchManagerError(f'不支持的地图库用途: {role}')
        if not os.path.isfile(pcd_path):
            raise LaunchManagerError(
                f'地图库中缺少 {name}.pcd，请先完成 FAST-LIO 建图')
        arguments = [f'map_file:={pcd_path}']
        if role == 'bundle' and not os.path.isfile(tomogram_path):
            raise LaunchManagerError(
                f'地图库中缺少 {name}.pickle，请先运行 PCD → PCT')
        arguments.append(f'tomogram_file:={tomogram_path}')
        return arguments, name

    @staticmethod
    def _resolve_parameter_arguments(
        profile: LaunchProfile,
        values: Optional[dict],
    ) -> tuple[list[str], dict[str, object]]:
        if values is None:
            return [], {}
        if not isinstance(values, dict):
            raise LaunchManagerError('Launch 参数必须是对象')
        specs = {
            argument.name: argument
            for argument in profile.configurable_arguments
        }
        unknown = sorted(set(values) - set(specs))
        if unknown:
            raise LaunchManagerError(
                f'当前模式不允许修改参数: {", ".join(unknown[:5])}')
        arguments = []
        accepted = {}
        for name, raw_value in values.items():
            serialized = specs[name].launch_value(raw_value)
            arguments.append(f'{name}:={serialized}')
            accepted[name] = raw_value
        return arguments, accepted

    def start(
        self,
        profile_id: str,
        map_name: Optional[str] = None,
        parameters: Optional[dict] = None,
    ):
        """Start one allowlisted profile, rejecting concurrent processes."""
        if not self.enabled:
            raise LaunchManagerError('Web Launch 控制未启用')
        profile = self._profiles.get(profile_id)
        if profile is None:
            raise LaunchManagerError('请从白名单中选择 Launch')
        if not profile.available:
            raise LaunchManagerError(
                profile.unavailable_reason or '当前 Launch 尚未完成配置'
            )
        arguments, selected_map = self._resolve_map_arguments(
            profile,
            map_name,
        )
        parameter_arguments, accepted_parameters = (
            self._resolve_parameter_arguments(profile, parameters)
        )
        arguments.extend(parameter_arguments)

        with self._lock:
            if self._shutdown:
                raise LaunchManagerError('Launch 管理器正在关闭')
            if self._process is not None and self._process.poll() is None:
                raise LaunchManagerError('已有 Launch 在运行，请先停止当前流程')
            command = self._command_factory(profile, arguments)
            environment = os.environ.copy()
            environment['RCUTILS_COLORIZED_OUTPUT'] = '0'
            environment['PYTHONUNBUFFERED'] = '1'
            self._state = 'starting'
            self._active_profile = profile
            self._active_map_name = selected_map
            self._active_parameters = accepted_parameters
            self._exit_code = None
            self._started_at = time.time()
            self._command = list(command)

            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    encoding='utf-8',
                    errors='replace',
                    bufsize=1,
                    env=environment,
                    start_new_session=True,
                )
            except (OSError, ValueError) as error:
                self._state = 'error'
                self._exit_code = -1
                self._append_log(f'Launch 启动失败: {error}', 'error')
                self._notify_status()
                raise LaunchManagerError(f'Launch 启动失败: {error}') from error

            self._process = process
            self._state = 'running'

        self._append_log(
            f'启动 {profile.label} (PID {process.pid})',
            'system',
        )
        self._append_log(f'命令: {" ".join(command)}', 'system')
        threading.Thread(
            target=self._read_output,
            args=(process,),
            name='nav2-launch-log',
            daemon=True,
        ).start()
        threading.Thread(
            target=self._watch_process,
            args=(process,),
            name='nav2-launch-watch',
            daemon=True,
        ).start()
        self._notify_status()

    def _read_output(self, process: subprocess.Popen):
        output = process.stdout
        if output is None:
            return
        try:
            for line in iter(output.readline, ''):
                self._append_log(line)
                self._handle_fatal_output(process, line)
        except (OSError, ValueError):
            pass
        finally:
            try:
                output.close()
            except OSError:
                pass

    def _handle_fatal_output(self, process: subprocess.Popen, line: str):
        normalized = _ANSI_ESCAPE.sub('', line).lower()
        if not any(pattern in normalized for pattern in _LIVOX_FATAL_OUTPUT):
            return
        with self._lock:
            if self._process is not process or self._state == 'error':
                return
            self._state = 'error'
        self._append_log(
            'Mid-360S 驱动初始化失败：请确认 MID360s_config.json 的 host_ip '
            '已经配置到本机有线网卡，并检查雷达 IP 与网线。',
            'error',
        )
        self._notify_status()
        threading.Thread(
            target=self._terminate_process,
            args=(process,),
            name='nav2-launch-fatal-stop',
            daemon=True,
        ).start()

    def _watch_process(self, process: subprocess.Popen):
        exit_code = process.wait()
        with self._lock:
            if self._process is not process:
                return
            previous_state = self._state
            self._exit_code = exit_code
            self._state = 'error' if previous_state == 'error' else 'exited'
        if previous_state == 'stopping':
            self._append_log(f'Launch 已停止，退出码 {exit_code}', 'system')
        elif previous_state == 'error':
            self._append_log(f'Launch 启动失败，退出码 {exit_code}', 'error')
        else:
            self._append_log(f'Launch 已退出，退出码 {exit_code}', 'system')
        self._notify_status()

    @staticmethod
    def _terminate_process(process: subprocess.Popen):
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=8.0)
            return
        except (OSError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=4.0)
            return
        except (OSError, subprocess.TimeoutExpired):
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except OSError:
            pass

    def stop(self) -> bool:
        """Begin stopping; return true when an external PCD save is required."""
        if not self.enabled:
            raise LaunchManagerError('Web Launch 控制未启用')
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                raise LaunchManagerError('当前没有正在运行的 Launch')
            if self._state in ('saving', 'stopping'):
                raise LaunchManagerError(
                    '地图正在保存或 Launch 已在停止中')
            save_before_stop = bool(
                self._active_profile and self._active_profile.save_before_stop)
            if save_before_stop:
                self._state = 'saving'
            else:
                self._state = 'stopping'
        if save_before_stop:
            self._append_log(
                '正在保存 FAST-LIO PCD；确认文件写入成功后自动停止…',
                'system',
            )
            self._notify_status()
            return True

        self._start_process_termination(process)
        return False

    def complete_save_before_stop(self, success: bool, message: str) -> bool:
        """Continue or cancel a pending stop after the PCD save response."""
        with self._lock:
            process = self._process
            running = process is not None and process.poll() is None
            if self._state != 'saving' or not running:
                return False
            self._state = 'stopping' if success else 'running'

        if not success:
            self._append_log(
                f'PCD 保存失败，建图继续运行：{message}', 'error')
            self._notify_status()
            return True

        self._append_log(f'PCD 保存成功：{message}', 'system')
        self._start_process_termination(process)
        return True

    def active_pcd_output_path(self) -> Optional[str]:
        """Return the allowlisted output path for an active PCD mapping run."""
        with self._lock:
            profile = self._active_profile
            map_name = self._active_map_name
        if profile is None or profile.map_role != 'pcd_output' or not map_name:
            return None
        return self._scanplanner_map_path(map_name, '.pcd')

    def _start_process_termination(self, process: subprocess.Popen):
        """Publish stopping state and terminate the selected process group."""
        with self._lock:
            self._state = 'stopping'
        self._append_log('正在停止当前 Launch…', 'system')
        self._notify_status()
        threading.Thread(
            target=self._terminate_process,
            args=(process,),
            name='nav2-launch-stop',
            daemon=True,
        ).start()

    def clear_logs(self):
        """Discard the retained log buffer without touching the process."""
        with self._lock:
            self._logs.clear()
        self._notify_status()

    def shutdown(self):
        """Stop the managed process synchronously during bridge shutdown."""
        with self._lock:
            self._shutdown = True
            process = self._process
            running = process is not None and process.poll() is None
            if running:
                self._state = 'stopping'
        if running:
            self._append_log('Web Bridge 关闭，正在停止受管 Launch…', 'system')
            self._terminate_process(process)
