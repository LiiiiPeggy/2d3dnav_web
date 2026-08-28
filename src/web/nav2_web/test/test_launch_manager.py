"""Tests for the allowlisted Web launch process manager."""

import os
from pathlib import Path
import sys
import tempfile
import time
import unittest

from nav2_web.launch_manager import LaunchManager
from nav2_web.launch_manager import LaunchManagerError
from nav2_web.launch_manager import LaunchProfile


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

    def test_disabled_manager_rejects_process_changes(self):
        """The default disabled state cannot mutate external processes."""
        manager = LaunchManager(enabled=False, map_directory=os.getcwd())
        with self.assertRaises(LaunchManagerError):
            manager.start('cartographer_mapping')
        with self.assertRaises(LaunchManagerError):
            manager.stop()


if __name__ == '__main__':
    unittest.main()
