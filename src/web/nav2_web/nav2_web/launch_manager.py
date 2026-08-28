"""Allowlisted ROS 2 launch process manager for the Nav2 Web console."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
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


class LaunchManagerError(ValueError):
    """A launch-control request was rejected before changing process state."""


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

    def public_dict(self) -> dict:
        """Return the fields that are safe to expose to Web clients."""
        return {
            'id': self.profile_id,
            'label': self.label,
            'description': self.description,
            'requires_map': self.requires_map,
            'requires_pbstream': self.requires_pbstream,
        }


def dog_launch_profiles() -> tuple[LaunchProfile, ...]:
    """Return the fixed launch allowlist for this robot workspace."""
    package = 'dog_cartographer_nav2_bringup'
    return (
        LaunchProfile(
            profile_id='cartographer_mapping',
            label='Cartographer 纯激光建图',
            description='纯激光扫描建图，Cartographer 负责 map/odom TF',
            package=package,
            launch_file='cartographer_mapping.launch.py',
            arguments=('start_web:=False', 'start_rf2o_prior:=False'),
        ),
        LaunchProfile(
            profile_id='cartographer_imu_mapping',
            label='Cartographer + IMU 建图',
            description='激光与 YIS320 IMU 融合建图，不启动 RF2O',
            package=package,
            launch_file='cartographer_mapping.launch.py',
            arguments=(
                'start_web:=False',
                'start_rf2o_prior:=False',
                'publish_imu_tf:=True',
                'cartographer_configuration_basename:='
                'ydlidar_cartographer_2d_imu.lua',
            ),
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
            label='图 SLAM 纯激光 + MPPI',
            description='加载 YAML 和同名 PBSTREAM，使用纯激光冻结图定位',
            package=package,
            launch_file='cartographer_mppi_navigation.launch.py',
            arguments=('start_web:=False',),
            requires_map=True,
            requires_pbstream=True,
        ),
        LaunchProfile(
            profile_id='cartographer_mppi_imu_navigation',
            label='图 SLAM + IMU + MPPI',
            description='加载 YAML/PBSTREAM，使用激光与 YIS320 IMU 定位',
            package=package,
            launch_file='cartographer_mppi_navigation.launch.py',
            arguments=(
                'start_web:=False',
                'publish_imu_tf:=True',
                'odom_use_imu_angular_velocity:=True',
                'cartographer_configuration_basename:='
                'ydlidar_cartographer_2d_localization_imu.lua',
            ),
            requires_map=True,
            requires_pbstream=True,
        ),
    )


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
        return [
            'ros2', 'launch', profile.package, profile.launch_file,
            *profile.arguments, *arguments,
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
        for name in names:
            if not name.endswith('.yaml') or name != os.path.basename(name):
                continue
            yaml_path = os.path.join(self.map_directory, name)
            if not os.path.isfile(yaml_path):
                continue
            stem = name[:-5]
            maps.append({
                'name': name,
                'label': stem,
                'has_pbstream': os.path.isfile(os.path.join(
                    self.map_directory, f'{stem}.pbstream'
                )),
            })
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

    def start(self, profile_id: str, map_name: Optional[str] = None):
        """Start one allowlisted profile, rejecting concurrent processes."""
        if not self.enabled:
            raise LaunchManagerError('Web Launch 控制未启用')
        profile = self._profiles.get(profile_id)
        if profile is None:
            raise LaunchManagerError('请从白名单中选择 Launch')
        arguments, selected_map = self._resolve_map_arguments(
            profile,
            map_name,
        )

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
        except (OSError, ValueError):
            pass
        finally:
            try:
                output.close()
            except OSError:
                pass

    def _watch_process(self, process: subprocess.Popen):
        exit_code = process.wait()
        with self._lock:
            if self._process is not process:
                return
            previous_state = self._state
            self._exit_code = exit_code
            self._state = 'exited'
        if previous_state == 'stopping':
            self._append_log(f'Launch 已停止，退出码 {exit_code}', 'system')
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

    def stop(self):
        """Request a graceful asynchronous stop of the active process."""
        if not self.enabled:
            raise LaunchManagerError('Web Launch 控制未启用')
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                raise LaunchManagerError('当前没有正在运行的 Launch')
            if self._state == 'stopping':
                raise LaunchManagerError('Launch 已在停止中')
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
