"""Tests for the FAST-LIO save-before-stop Web workflow."""

import os
from types import SimpleNamespace

from nav2_web.bridge import Nav2WebBridge


class _Future:
    def __init__(self, response):
        self._response = response

    def result(self):
        return self._response

    def add_done_callback(self, callback):
        callback(self)


class _Client:
    def __init__(self, response=None, ready=True):
        self._response = response
        self._ready = ready

    def service_is_ready(self):
        return self._ready

    def call_async(self, _request):
        return _Future(self._response)


class _Manager:
    def __init__(self, path):
        self._path = path
        self.completions = []

    def stop(self):
        return True

    def active_pcd_output_path(self):
        return self._path

    def complete_save_before_stop(self, success, message):
        self.completions.append((success, message))
        return True


class _Socket:
    def __init__(self):
        self.messages = []

    def send_json(self, message):
        self.messages.append(message)


def _bridge(path, client):
    bridge = object.__new__(Nav2WebBridge)
    bridge._launch_manager = _Manager(path)
    bridge._fastlio_map_save_client = client
    bridge.fastlio_map_save_service = '/map_save'
    bridge._servers = SimpleNamespace(websocket=_Socket())
    return bridge


def test_successful_fastlio_save_checks_file_before_stopping(tmp_path):
    output = tmp_path / 'phone_map.pcd'
    output.write_bytes(b'pcd')
    response = SimpleNamespace(success=True, message='saved 3 points')
    bridge = _bridge(str(output), _Client(response))

    bridge._stop_managed_launch()

    assert bridge._launch_manager.completions[0][0] is True
    assert bridge._servers.websocket.messages[-1]['state'] == 'succeeded'
    assert os.path.basename(str(output)) in (
        bridge._servers.websocket.messages[-1]['message'])


def test_unavailable_fastlio_save_service_keeps_mapping_running(tmp_path):
    output = tmp_path / 'phone_map.pcd'
    bridge = _bridge(str(output), _Client(ready=False))

    bridge._stop_managed_launch()

    assert bridge._launch_manager.completions[0][0] is False
    assert bridge._servers.websocket.messages[-1]['state'] == 'error'
    assert '建图保持运行' in bridge._servers.websocket.messages[-1]['message']
