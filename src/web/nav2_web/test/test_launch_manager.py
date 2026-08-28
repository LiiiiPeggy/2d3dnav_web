"""Tests for the allowlisted Web launch process manager."""

import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

from nav2_web.launch_manager import LaunchManager
from nav2_web.launch_manager import LaunchManagerError
from nav2_web.launch_manager import LaunchArgument
from nav2_web.launch_manager import LaunchProfile
from nav2_web.launch_manager import scanplanner_launch_profiles


def _wait_until(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class LaunchManagerTest(unittest.TestCase):
    """Exercise validation, logging, and process-group lifecycle."""

    def test_map_selection_stays_inside_allowlisted_directory(self):
        """A navigation request can only use a YAML inside maps."""
        with tempfile.TemporaryDirectory() as directory:
            map_path = Path(directory, 'floor.yaml')
            map_path.write_text('image: floor.pgm\n', encoding='utf-8')
            profile = LaunchProfile(
                profile_id='navigation',
                label='Navigation',
                description='test',
                package='test_package',
                launch_file='test.launch.py',
                requires_map=True,
            )
            commands = []
            manager = LaunchManager(
                enabled=True,
                map_directory=directory,
                profiles=(profile,),
                command_factory=lambda selected, arguments: (
                    commands.append((selected, arguments))
                    or [sys.executable, '-c', 'pass']
                ),
            )
            manager.start('navigation', 'floor.yaml')
            self.assertTrue(_wait_until(
                lambda: manager.snapshot()['active']['state'] == 'exited'
            ))
            self.assertEqual(commands[0][0], profile)
            self.assertEqual(commands[0][1], [f'map:={map_path}'])
            with self.assertRaises(LaunchManagerError):
                manager.start('navigation', '../floor.yaml')
            manager.shutdown()

    def test_scanplanner_map_library_lists_and_resolves_bundles(self):
        """The App selects a safe PCD/PCT pair by name, never by raw path."""
        with tempfile.TemporaryDirectory() as directory:
            pcd = Path(directory, 'building2_9.pcd')
            tomogram = Path(directory, 'building2_9.pickle')
            pcd.write_bytes(b'pcd')
            tomogram.write_bytes(b'pickle')
            Path(directory, 'pcd_only.pcd').write_bytes(b'pcd')
            profile = LaunchProfile(
                profile_id='pct_demo', label='PCT demo', description='test',
                package='test_package', launch_file='test.launch.py',
                map_role='bundle', default_map_name='building2_9',
            )
            commands = []
            manager = LaunchManager(
                enabled=True, map_directory=directory, profiles=(profile,),
                command_factory=lambda _profile, arguments: (
                    commands.append(arguments)
                    or [sys.executable, '-c', 'pass']),
            )

            catalog = {
                item['name']: item for item in manager.snapshot()['maps']
                if item.get('kind') == 'scanplanner'
            }
            self.assertTrue(catalog['building2_9']['has_pcd'])
            self.assertTrue(catalog['building2_9']['has_tomogram'])
            self.assertTrue(catalog['pcd_only']['has_pcd'])
            self.assertFalse(catalog['pcd_only']['has_tomogram'])

            manager.start('pct_demo', 'building2_9.pcd')
            self.assertTrue(_wait_until(
                lambda: manager.snapshot()['active']['state'] == 'exited'))
            self.assertEqual(commands[0], [
                f'map_file:={pcd}', f'tomogram_file:={tomogram}'])
            with self.assertRaisesRegex(LaunchManagerError, '地图名只能'):
                manager.start('pct_demo', '../building2_9')
            with self.assertRaisesRegex(LaunchManagerError, '缺少.*pickle'):
                manager.start('pct_demo', 'pcd_only')
            manager.shutdown()

    def test_process_logs_are_retained_and_process_group_can_stop(self):
        """Output is retained and SIGINT stops the complete process group."""
        with tempfile.TemporaryDirectory() as directory:
            profile = LaunchProfile(
                profile_id='long_running',
                label='Long running',
                description='test',
                package='test_package',
                launch_file='test.launch.py',
            )
            manager = LaunchManager(
                enabled=True,
                map_directory=directory,
                profiles=(profile,),
                command_factory=lambda _profile, _arguments: [
                    sys.executable,
                    '-u',
                    '-c',
                    'import time; print("ready", flush=True); time.sleep(30)',
                ],
            )
            manager.start('long_running')
            self.assertTrue(_wait_until(lambda: any(
                'ready' in entry['line']
                for entry in manager.snapshot()['logs']
            )))
            self.assertTrue(manager.snapshot()['active']['running'])
            with self.assertRaises(LaunchManagerError):
                manager.start('long_running')
            manager.stop()
            self.assertTrue(_wait_until(
                lambda: not manager.snapshot()['active']['running'],
                timeout=10.0,
            ))
            self.assertIn(
                manager.snapshot()['active']['state'],
                ('stopping', 'exited'),
            )
            manager.shutdown()

    def test_fastlio_mapping_only_stops_after_successful_pcd_save(self):
        """A failed map save keeps mapping alive; a successful one may stop it."""
        with tempfile.TemporaryDirectory() as directory:
            profile = LaunchProfile(
                profile_id='mapping',
                label='FAST-LIO mapping',
                description='test',
                package='test_package',
                launch_file='test.launch.py',
                map_role='pcd_output',
                save_before_stop=True,
            )
            manager = LaunchManager(
                enabled=True,
                map_directory=directory,
                profiles=(profile,),
                command_factory=lambda _profile, _arguments: [
                    sys.executable,
                    '-u',
                    '-c',
                    'import time; print("mapping", flush=True); time.sleep(30)',
                ],
            )
            manager.start('mapping', 'new_site')
            self.assertTrue(_wait_until(
                lambda: manager.snapshot()['active']['running']))
            self.assertEqual(
                manager.active_pcd_output_path(),
                str(Path(directory, 'new_site.pcd')),
            )

            self.assertTrue(manager.stop())
            self.assertEqual(
                manager.snapshot()['active']['state'], 'saving')
            self.assertTrue(manager.complete_save_before_stop(
                False, 'no accumulated points'))
            self.assertEqual(
                manager.snapshot()['active']['state'], 'running')
            self.assertTrue(manager.snapshot()['active']['running'])

            self.assertTrue(manager.stop())
            self.assertTrue(manager.complete_save_before_stop(
                True, 'new_site.pcd'))
            self.assertTrue(_wait_until(
                lambda: not manager.snapshot()['active']['running'],
                timeout=10.0,
            ))
            self.assertEqual(
                manager.snapshot()['active']['state'], 'exited')
            manager.shutdown()

    def test_disabled_manager_rejects_process_changes(self):
        """The default disabled state cannot mutate external processes."""
        manager = LaunchManager(enabled=False, map_directory=os.getcwd())
        with self.assertRaises(LaunchManagerError):
            manager.start('cartographer_mapping')
        with self.assertRaises(LaunchManagerError):
            manager.stop()

    def test_app_arguments_are_typed_allowlisted_and_bounded(self):
        """A client can tune declared values but cannot append launch keys."""
        profile = LaunchProfile(
            profile_id='configured', label='Configured', description='test',
            package='test_package', launch_file='test.launch.py',
            configurable_arguments=(
                LaunchArgument(
                    name='pitch', label='Pitch', kind='float', default=0.0,
                    group='TF', minimum=-1.0, maximum=1.0),
                LaunchArgument(
                    name='driver', label='Driver', kind='bool', default=True,
                    group='Components'),
            ),
        )
        commands = []
        manager = LaunchManager(
            enabled=True, map_directory=os.getcwd(), profiles=(profile,),
            command_factory=lambda _profile, arguments: (
                commands.append(arguments) or [sys.executable, '-c', 'pass']),
        )
        manager.start('configured', parameters={
            'pitch': '-0.261799', 'driver': False})
        self.assertTrue(_wait_until(
            lambda: manager.snapshot()['active']['state'] == 'exited'))
        self.assertEqual(commands[0], ['pitch:=-0.261799', 'driver:=false'])
        with self.assertRaisesRegex(LaunchManagerError, '不允许修改'):
            manager.start('configured', parameters={'shell': 'anything'})
        with self.assertRaisesRegex(LaunchManagerError, '不能大于'):
            manager.start('configured', parameters={'pitch': 9.0})
        manager.shutdown()

    def test_livox_initialization_failure_is_not_left_running(self):
        """A stuck Livox process becomes an explicit launch error."""
        with tempfile.TemporaryDirectory() as directory:
            profile = LaunchProfile(
                profile_id='livox',
                label='Mid-360S',
                description='test',
                package='test_package',
                launch_file='test.launch.py',
            )
            manager = LaunchManager(
                enabled=True,
                map_directory=directory,
                profiles=(profile,),
                command_factory=lambda _profile, _arguments: [
                    sys.executable,
                    '-u',
                    '-c',
                    'import time; print("Init lds lidar fail!", flush=True); '
                    'time.sleep(30)',
                ],
            )
            manager.start('livox')
            self.assertTrue(_wait_until(
                lambda: manager.snapshot()['active']['state'] == 'error',
                timeout=2.0,
            ))
            self.assertTrue(_wait_until(
                lambda: not manager.snapshot()['active']['running'],
                timeout=10.0,
            ))
            self.assertTrue(any(
                'Mid-360S 驱动初始化失败' in entry['line']
                for entry in manager.snapshot()['logs']
            ))
            manager.shutdown()

    def test_scanplanner_profiles_are_safe_and_complete(self):
        """Physical modes cover each stage and control is explicitly marked."""
        with tempfile.TemporaryDirectory() as directory:
            keypoints = Path(directory, 'keypoints.yaml')
            reference = Path(directory, 'reference.yaml')
            prior_map = Path(directory, 'scanplanner_map.pcd')
            tomogram = Path(directory, 'scanplanner_map.pickle')
            keypoints.write_text('scan_planner_node: {}\n', encoding='utf-8')
            reference.write_text(
                'reference_path_publisher: {}\n', encoding='utf-8')
            prior_map.write_bytes(b'pcd')
            tomogram.write_bytes(b'pickle')
            profiles = scanplanner_launch_profiles(
                str(keypoints), str(reference),
                str(prior_map), str(tomogram))
        self.assertEqual(len(profiles), 13)
        self.assertEqual(profiles[0].profile_id, 'mid360s_terminal')
        self.assertIn('Mid-360S', profiles[0].label)
        self.assertEqual(
            [profile.navigation_mode for profile in profiles[2:8]],
            [1, 2, 3, 1, 2, 3],
        )
        self.assertTrue(all(profile.available for profile in profiles))
        for profile in profiles[:8]:
            self.assertIn('start_livox_driver:=true', profile.arguments)
            self.assertIn('start_mobile_3d:=false', profile.arguments)
        for profile in profiles[2:5]:
            self.assertFalse(profile.dangerous)
            self.assertIn('enable_control:=false', profile.arguments)
        for profile in profiles[5:]:
            if profile.profile_id.startswith('scanplanner_mode'):
                self.assertTrue(profile.dangerous)
                self.assertIn('enable_control:=true', profile.arguments)
        self.assertEqual(
            [profile.profile_id for profile in profiles[8:]],
            [
                'fastlio_global_mapping',
                'pct_tomogram_build',
                'pct_offline_demo',
                'pct_scanplanner_mode3_preview',
                'pct_scanplanner_mode3_control',
            ],
        )
        self.assertFalse(profiles[11].dangerous)
        self.assertTrue(profiles[12].dangerous)
        self.assertTrue(profiles[8].save_before_stop)
        self.assertFalse(any(
            profile.save_before_stop for profile in profiles[:8] + profiles[9:]
        ))
        # The phone exposes RViz and essential PCT heights, while physical
        # extrinsics and advanced ICP tuning stay in launch/YAML.
        for profile in profiles:
            names = [
                argument.name for argument in profile.configurable_arguments
            ]
            self.assertIn('start_rviz', names)
        self.assertEqual(
            [argument.name for argument in profiles[9].configurable_arguments],
            ['start_rviz', 'ground_height', 'slice_height'],
        )
        self.assertEqual(
            [argument.name for argument in profiles[10].configurable_arguments],
            ['start_rviz', 'body_height'],
        )
        self.assertEqual(
            [argument.name for argument in profiles[11].configurable_arguments],
            ['start_rviz', 'body_height'],
        )

    def test_dynamic_launch_argument_replaces_fixed_default(self):
        """An App RViz toggle replaces, rather than duplicates, the default."""
        profile = LaunchProfile(
            profile_id='rviz', label='RViz', description='test',
            package='test_package', launch_file='test.launch.py',
            arguments=('start_rviz:=false', 'other:=fixed'),
        )
        self.assertEqual(
            LaunchManager._ros2_command(profile, ['start_rviz:=true']),
            [
                'ros2', 'launch', 'test_package', 'test.launch.py',
                'other:=fixed', 'start_rviz:=true',
            ],
        )

    def test_missing_prior_map_is_checked_after_app_selection(self):
        """Map-dependent modes stay visible but reject an incomplete bundle."""
        with tempfile.TemporaryDirectory() as directory:
            map_file = str(Path(directory, 'not_built_yet.pcd'))
            tomogram_file = str(Path(directory, 'not_built_yet.pickle'))
            profiles = scanplanner_launch_profiles(
                map_file=map_file,
                tomogram_file=tomogram_file,
            )

        profile_by_id = {
            profile.profile_id: profile for profile in profiles
        }
        self.assertTrue(profile_by_id['fastlio_global_mapping'].available)
        self.assertTrue(profile_by_id['pct_tomogram_build'].available)
        self.assertTrue(profile_by_id['pct_offline_demo'].available)
        self.assertTrue(
            profile_by_id['pct_scanplanner_mode3_preview'].available)
        self.assertTrue(
            profile_by_id['pct_scanplanner_mode3_control'].available)
        manager = LaunchManager(
            enabled=True,
            map_directory=directory,
            profiles=profiles,
            command_factory=lambda _profile, _arguments: [
                sys.executable, '-c', 'pass'],
        )
        with self.assertRaisesRegex(LaunchManagerError, '缺少.*pcd'):
            manager.start('pct_offline_demo', 'not_built_yet')
        manager.shutdown()

    def test_scanplanner_mode_two_requires_configured_waypoints(self):
        """Mode 2 is visible but cannot start with invented real coordinates."""
        profiles = scanplanner_launch_profiles()
        mode_two = next(
            profile for profile in profiles
            if profile.profile_id == 'scanplanner_mode2_preview'
        )
        self.assertFalse(mode_two.available)
        manager = LaunchManager(
            enabled=True,
            map_directory=os.getcwd(),
            profiles=profiles,
            command_factory=lambda _profile, _arguments: [
                sys.executable, '-c', 'pass'],
        )
        with self.assertRaisesRegex(LaunchManagerError, 'keypoints_file'):
            manager.start(mode_two.profile_id)
        manager.shutdown()


if __name__ == '__main__':
    unittest.main()
