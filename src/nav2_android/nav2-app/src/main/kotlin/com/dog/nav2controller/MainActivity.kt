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

import android.os.Bundle
import android.view.WindowManager
import androidx.activity.ComponentActivity
import androidx.activity.compose.BackHandler
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.viewModels
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.key
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle

class MainActivity : ComponentActivity() {
    private val viewModel: Nav2ViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        setContent {
            Nav2ControllerTheme {
                Nav2ControllerApp(viewModel)
            }
        }
    }
}

@Composable
private fun Nav2ControllerTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = darkColorScheme(
            primary = Color(0xFF28D7A1),
            secondary = Color(0xFF48B7FF),
            background = Color(0xFF071019),
            surface = Color(0xFF101D28),
        ),
        content = content,
    )
}

@Composable
private fun Nav2ControllerApp(viewModel: Nav2ViewModel) {
    val endpoint by viewModel.endpoint.collectAsStateWithLifecycle()
    var showSettings by rememberSaveable { mutableStateOf(endpoint == null) }
    var bridgeConnected by remember { mutableStateOf(false) }

    BackHandler(enabled = !showSettings && endpoint != null) {
        showSettings = true
    }

    BoxWithConstraints(
        modifier = Modifier
            .fillMaxSize()
            .background(MaterialTheme.colorScheme.background),
    ) {
        endpoint?.let { selectedEndpoint ->
            key(selectedEndpoint) {
                Nav2WebView(
                    endpoint = selectedEndpoint,
                    onConnectionChanged = { bridgeConnected = it },
                )
            }

            Button(
                onClick = { showSettings = true },
                modifier = Modifier
                    .align(Alignment.TopEnd)
                    .then(
                        if (maxWidth <= maxHeight) {
                            Modifier
                                .statusBarsPadding()
                                .padding(top = 6.dp, end = 6.dp)
                        } else {
                            Modifier.padding(top = 6.dp, end = 56.dp)
                        },
                    ),
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (bridgeConnected) {
                        Color(0xCC126C57)
                    } else {
                        Color(0xCCD44848)
                    },
                ),
                contentPadding = ButtonDefaults.ContentPadding,
            ) {
                Text(
                    text = if (bridgeConnected) "已连接" else "连接设置",
                    fontSize = 12.sp,
                )
            }
        } ?: EmptyConnectionScreen(onConnect = { showSettings = true })

        if (showSettings) {
            ConnectionDialog(
                current = endpoint,
                canDismiss = endpoint != null,
                onDismiss = { showSettings = false },
                onSave = {
                    viewModel.saveEndpoint(it)
                    bridgeConnected = false
                    showSettings = false
                },
            )
        }
    }
}

@Composable
private fun EmptyConnectionScreen(onConnect: () -> Unit) {
    Column(
        modifier = Modifier
            .fillMaxSize()
            .padding(32.dp),
        verticalArrangement = Arrangement.Center,
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("planner", style = MaterialTheme.typography.headlineLarge)
        Text(
            text = "连接机器人后，可查看 3D 场景、启动实机流程并读取终端日志。",
            modifier = Modifier.padding(vertical = 18.dp),
            color = Color(0xFFB8C6D1),
        )
        Button(onClick = onConnect) { Text("配置机器人连接") }
    }
}

@Composable
private fun ConnectionDialog(
    current: RobotEndpoint?,
    canDismiss: Boolean,
    onDismiss: () -> Unit,
    onSave: (RobotEndpoint) -> Unit,
) {
    var host by remember(current) { mutableStateOf(current?.host.orEmpty()) }
    var httpPort by remember(current) {
        mutableStateOf((current?.httpPort ?: RobotEndpoint.DEFAULT_HTTP_PORT).toString())
    }
    var webSocketPort by remember(current) {
        mutableStateOf(
            (current?.webSocketPort ?: RobotEndpoint.DEFAULT_WEBSOCKET_PORT).toString(),
        )
    }
    var errorMessage by remember { mutableStateOf<String?>(null) }

    AlertDialog(
        onDismissRequest = { if (canDismiss) onDismiss() },
        title = { Text("机器人连接") },
        text = {
            Column(
                modifier = Modifier.verticalScroll(rememberScrollState()),
                verticalArrangement = Arrangement.spacedBy(10.dp),
            ) {
                Text("机器人端先运行 planner_app.launch.py，App 将自动连接并保持在线。")
                OutlinedTextField(
                    value = host,
                    onValueChange = {
                        host = it
                        errorMessage = null
                    },
                    label = { Text("机器人 IP / 主机名") },
                    placeholder = { Text("例如 10.10.10.186") },
                    singleLine = true,
                    modifier = Modifier.fillMaxWidth(),
                )
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    horizontalArrangement = Arrangement.spacedBy(10.dp),
                ) {
                    OutlinedTextField(
                        value = httpPort,
                        onValueChange = {
                            httpPort = it
                            errorMessage = null
                        },
                        label = { Text("HTTP") },
                        singleLine = true,
                        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                        modifier = Modifier.weight(1f),
                    )
                    OutlinedTextField(
                        value = webSocketPort,
                        onValueChange = {
                            webSocketPort = it
                            errorMessage = null
                        },
                        label = { Text("WebSocket") },
                        singleLine = true,
                        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
                        modifier = Modifier.weight(1f),
                    )
                }
                errorMessage?.let {
                    Text(it, color = MaterialTheme.colorScheme.error)
                }
            }
        },
        confirmButton = {
            Button(
                onClick = {
                    RobotEndpoint.parse(host, httpPort, webSocketPort)
                        .onSuccess(onSave)
                        .onFailure { errorMessage = it.message ?: "连接配置不正确" }
                },
            ) {
                Text("保存并连接")
            }
        },
        dismissButton = if (canDismiss) {
            { TextButton(onClick = onDismiss) { Text("取消") } }
        } else {
            null
        },
    )
}
