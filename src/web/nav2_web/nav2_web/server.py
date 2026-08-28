"""Small dependency-free HTTP and WebSocket servers for the Nav2 UI."""

from __future__ import annotations

import base64
from functools import partial
import hashlib
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
import os
import socket
import struct
import threading
from typing import Callable, Optional


_WEBSOCKET_GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


class _StaticHandler(SimpleHTTPRequestHandler):
    """Serve the installed Web UI and expose the WebSocket port."""

    ws_port = 8891

    def do_GET(self):  # noqa: N802 (stdlib callback name)
        if self.path.split('?', 1)[0] == '/config.json':
            payload = json.dumps({'ws_port': self.ws_port}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(payload)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(payload)
            return
        super().do_GET()

    def end_headers(self):
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Referrer-Policy', 'no-referrer')
        super().end_headers()

    def log_message(self, _format, *_args):
        return


class HttpServer:
    def __init__(self, root: str, port: int, ws_port: int):
        if not os.path.isfile(os.path.join(root, 'index.html')):
            raise FileNotFoundError(f'Web root has no index.html: {root}')

        handler_type = type(
            'ConfiguredStaticHandler',
            (_StaticHandler,),
            {'ws_port': ws_port},
        )
        handler = partial(handler_type, directory=root)
        self._server = ThreadingHTTPServer(('0.0.0.0', port), handler)
        self._server.daemon_threads = True
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name='nav2-web-http',
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def stop(self):
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


class _WebSocketClient:
    def __init__(self, connection: socket.socket):
        self.connection = connection
        self.send_lock = threading.Lock()

    def send_frame(self, payload: bytes, opcode: int = 0x1):
        length = len(payload)
        if length < 126:
            header = struct.pack('!BB', 0x80 | opcode, length)
        elif length <= 0xFFFF:
            header = struct.pack('!BBH', 0x80 | opcode, 126, length)
        else:
            header = struct.pack('!BBQ', 0x80 | opcode, 127, length)
        with self.send_lock:
            self.connection.sendall(header + payload)


class WebSocketHub:
    """Minimal RFC 6455 text WebSocket hub for modern browsers."""

    def __init__(
        self,
        port: int,
        on_message: Callable[[str], None],
        on_connect: Optional[Callable[[_WebSocketClient], None]] = None,
    ):
        self._port = port
        self._on_message = on_message
        self._on_connect = on_connect
        self._running = threading.Event()
        self._server_socket: Optional[socket.socket] = None
        self._accept_thread: Optional[threading.Thread] = None
        self._clients: set[_WebSocketClient] = set()
        self._clients_lock = threading.Lock()

    @staticmethod
    def _read_exact(connection: socket.socket, size: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < size:
            chunk = connection.recv(size - len(chunks))
            if not chunk:
                raise ConnectionError('WebSocket disconnected')
            chunks.extend(chunk)
        return bytes(chunks)

    @staticmethod
    def _handshake(connection: socket.socket):
        request = bytearray()
        while b'\r\n\r\n' not in request and len(request) < 16384:
            chunk = connection.recv(2048)
            if not chunk:
                raise ConnectionError('Incomplete WebSocket handshake')
            request.extend(chunk)

        headers = {}
        lines = request.decode('latin-1').split('\r\n')
        for line in lines[1:]:
            if ':' in line:
                key, value = line.split(':', 1)
                headers[key.strip().lower()] = value.strip()

        websocket_key = headers.get('sec-websocket-key')
        if not websocket_key:
            raise ValueError('Missing Sec-WebSocket-Key')

        digest = hashlib.sha1(
            (websocket_key + _WEBSOCKET_GUID).encode('ascii')
        ).digest()
        accept_key = base64.b64encode(digest).decode('ascii')
        response = (
            'HTTP/1.1 101 Switching Protocols\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            f'Sec-WebSocket-Accept: {accept_key}\r\n\r\n'
        )
        connection.sendall(response.encode('ascii'))

    def start(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('0.0.0.0', self._port))
        server_socket.listen(8)
        server_socket.settimeout(0.5)
        self._server_socket = server_socket
        self._running.set()
        self._accept_thread = threading.Thread(
            target=self._accept_loop,
            name='nav2-web-ws',
            daemon=True,
        )
        self._accept_thread.start()

    def stop(self):
        self._running.clear()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        if self._accept_thread is not None:
            self._accept_thread.join(timeout=2.0)
        with self._clients_lock:
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            try:
                client.connection.close()
            except OSError:
                pass

    def _accept_loop(self):
        while self._running.is_set():
            try:
                connection, _address = self._server_socket.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            connection.settimeout(10.0)
            threading.Thread(
                target=self._client_loop,
                args=(connection,),
                name='nav2-web-client',
                daemon=True,
            ).start()

    def _client_loop(self, connection: socket.socket):
        client = _WebSocketClient(connection)
        try:
            self._handshake(connection)
            connection.settimeout(None)
            with self._clients_lock:
                self._clients.add(client)
            if self._on_connect is not None:
                self._on_connect(client)

            while self._running.is_set():
                header = self._read_exact(connection, 2)
                opcode = header[0] & 0x0F
                masked = bool(header[1] & 0x80)
                length = header[1] & 0x7F
                if length == 126:
                    length = struct.unpack('!H', self._read_exact(connection, 2))[0]
                elif length == 127:
                    length = struct.unpack('!Q', self._read_exact(connection, 8))[0]
                if length > 8 * 1024 * 1024:
                    raise ValueError('WebSocket frame is too large')
                mask = self._read_exact(connection, 4) if masked else b''
                payload = bytearray(self._read_exact(connection, length))
                if masked:
                    for index in range(length):
                        payload[index] ^= mask[index % 4]

                if opcode == 0x8:
                    break
                if opcode == 0x9:
                    client.send_frame(bytes(payload), opcode=0xA)
                    continue
                if opcode == 0x1:
                    self._on_message(payload.decode('utf-8'))
        except (ConnectionError, OSError, UnicodeDecodeError, ValueError):
            pass
        finally:
            with self._clients_lock:
                self._clients.discard(client)
            try:
                connection.close()
            except OSError:
                pass

    def send_json(self, message: dict, client: Optional[_WebSocketClient] = None):
        payload = json.dumps(
            message,
            ensure_ascii=False,
            separators=(',', ':'),
        ).encode('utf-8')
        targets = [client] if client is not None else self.clients()
        for target in targets:
            try:
                target.send_frame(payload)
            except OSError:
                with self._clients_lock:
                    self._clients.discard(target)
                try:
                    target.connection.close()
                except OSError:
                    pass

    def clients(self) -> list[_WebSocketClient]:
        with self._clients_lock:
            return list(self._clients)


class WebServers:
    def __init__(
        self,
        root: str,
        http_port: int,
        ws_port: int,
        on_message: Callable[[str], None],
        on_connect: Callable[[_WebSocketClient], None],
    ):
        mimetypes.add_type('application/javascript', '.js')
        self.http = HttpServer(root, http_port, ws_port)
        self.websocket = WebSocketHub(ws_port, on_message, on_connect)

    def start(self):
        self.websocket.start()
        try:
            self.http.start()
        except Exception:
            self.websocket.stop()
            raise

    def stop(self):
        self.http.stop()
        self.websocket.stop()
