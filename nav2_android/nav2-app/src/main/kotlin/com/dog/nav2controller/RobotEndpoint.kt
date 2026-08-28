/*
 * Copyright 2026 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.dog.nav2controller

import java.net.URI

data class RobotEndpoint(
    val host: String,
    val httpPort: Int = DEFAULT_HTTP_PORT,
    val webSocketPort: Int = DEFAULT_WEBSOCKET_PORT,
) {
    val httpOrigin: String
        get() = "http://${hostForUrl(host)}:$httpPort/"

    companion object {
        const val DEFAULT_HTTP_PORT = 8081
        const val DEFAULT_WEBSOCKET_PORT = 8891

        fun parse(
            hostInput: String,
            httpPortInput: String,
            webSocketPortInput: String,
        ): Result<RobotEndpoint> = runCatching {
            val rawHost = hostInput.trim()
            require(rawHost.isNotEmpty()) { "请输入机器人 IP 或主机名" }

            val candidate = if (rawHost.contains("://")) rawHost else "http://$rawHost"
            val uri = URI(candidate)
            require(uri.scheme.equals("http", ignoreCase = true) ||
                uri.scheme.equals("https", ignoreCase = true)) {
                "机器人地址只支持 http:// 或直接填写 IP/主机名"
            }
            val host = uri.host?.trim()?.takeIf(String::isNotEmpty)
                ?: throw IllegalArgumentException("机器人地址格式不正确")
            require(uri.userInfo == null && uri.query == null && uri.fragment == null) {
                "机器人地址不能包含账号、查询参数或片段"
            }
            require(uri.path.isNullOrEmpty() || uri.path == "/") {
                "机器人地址只需要 IP 或主机名，不要填写路径"
            }

            val httpPort = parsePort(
                uri.port.takeIf { it > 0 }?.toString() ?: httpPortInput,
                DEFAULT_HTTP_PORT,
                "HTTP",
            )
            val webSocketPort = parsePort(
                webSocketPortInput,
                DEFAULT_WEBSOCKET_PORT,
                "WebSocket",
            )
            RobotEndpoint(host, httpPort, webSocketPort)
        }

        private fun parsePort(input: String, defaultValue: Int, label: String): Int {
            val value = input.trim()
            val port = if (value.isEmpty()) {
                defaultValue
            } else {
                value.toIntOrNull()
                    ?: throw IllegalArgumentException("$label 端口必须是数字")
            }
            require(port in 1..65535) { "$label 端口必须在 1–65535 之间" }
            return port
        }

        private fun hostForUrl(host: String): String =
            if (':' in host && !host.startsWith('[')) "[$host]" else host
    }
}
