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

import android.content.Context

class ConnectionStore(context: Context) {
    private val preferences = context.getSharedPreferences(PREFERENCES_NAME, Context.MODE_PRIVATE)

    fun read(): RobotEndpoint? {
        val host = preferences.getString(KEY_HOST, null)?.takeIf(String::isNotBlank) ?: return null
        return RobotEndpoint(
            host = host,
            httpPort = preferences.getInt(KEY_HTTP_PORT, RobotEndpoint.DEFAULT_HTTP_PORT),
            webSocketPort = preferences.getInt(
                KEY_WEBSOCKET_PORT,
                RobotEndpoint.DEFAULT_WEBSOCKET_PORT,
            ),
        )
    }

    fun write(endpoint: RobotEndpoint) {
        preferences.edit()
            .putString(KEY_HOST, endpoint.host)
            .putInt(KEY_HTTP_PORT, endpoint.httpPort)
            .putInt(KEY_WEBSOCKET_PORT, endpoint.webSocketPort)
            .apply()
    }

    private companion object {
        const val PREFERENCES_NAME = "nav2_connection"
        const val KEY_HOST = "host"
        const val KEY_HTTP_PORT = "http_port"
        const val KEY_WEBSOCKET_PORT = "websocket_port"
    }
}
