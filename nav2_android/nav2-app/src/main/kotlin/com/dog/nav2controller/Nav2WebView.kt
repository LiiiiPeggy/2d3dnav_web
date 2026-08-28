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

import android.annotation.SuppressLint
import android.content.res.AssetManager
import android.graphics.Color
import android.os.Handler
import android.os.Looper
import android.webkit.JavascriptInterface
import android.webkit.WebChromeClient
import android.webkit.WebSettings
import android.webkit.WebView
import android.webkit.WebViewClient
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.ui.Modifier
import androidx.compose.ui.viewinterop.AndroidView

class Nav2JavascriptBridge(
    private val onConnectionChanged: (Boolean) -> Unit,
) {
    private val mainHandler = Handler(Looper.getMainLooper())

    @JavascriptInterface
    fun connectionChanged(connected: Boolean) {
        mainHandler.post { onConnectionChanged(connected) }
    }
}

private fun AssetManager.readNav2Asset(fileName: String): String =
    open("nav2_web/$fileName").bufferedReader(Charsets.UTF_8).use { it.readText() }

internal fun buildBundledNav2Page(
    html: String,
    stylesheet: String,
    script: String,
    webSocketPort: Int,
): String {
    val stylesheetTag = Regex(
        """<link\s+rel="stylesheet"\s+href="style\.css(?:\?[^"]*)?">""",
    )
    val scriptTag = Regex(
        """<script\s+src="app\.js(?:\?[^"]*)?"></script>""",
    )
    check(stylesheetTag.containsMatchIn(html) && scriptTag.containsMatchIn(html)) {
        "The bundled Nav2 index does not contain the expected asset tags"
    }
    val pageWithStyles = stylesheetTag.replace(html) {
        "<style>\n$stylesheet\n</style>"
    }
    return scriptTag.replace(pageWithStyles) {
        """
                <script>window.NAV2_ANDROID_WS_PORT = $webSocketPort;</script>
                <script>
                $script
                </script>
        """.trimIndent()
    }
}

/**
 * Builds one self-contained page so WebView never has to fetch APK assets through HTTP request
 * interception. The robot HTTP origin is still used as the page base URL, which makes config.json
 * and the WebSocket connect to the selected robot while the visible UI remains available offline.
 */
private fun AssetManager.bundledNav2Page(webSocketPort: Int): String {
    return buildBundledNav2Page(
        html = readNav2Asset("index.html"),
        stylesheet = readNav2Asset("style.css"),
        script = readNav2Asset("app.js"),
        webSocketPort = webSocketPort,
    )
}

@SuppressLint("SetJavaScriptEnabled")
@Composable
fun Nav2WebView(
    endpoint: RobotEndpoint,
    onConnectionChanged: (Boolean) -> Unit,
    modifier: Modifier = Modifier,
) {
    val currentOnConnectionChanged = rememberUpdatedState(onConnectionChanged)
    val bridge = remember {
        Nav2JavascriptBridge { connected ->
            currentOnConnectionChanged.value(connected)
        }
    }

    AndroidView(
        factory = { context ->
            val bundledPage = context.assets.bundledNav2Page(endpoint.webSocketPort)
            WebView(context).apply {
                setBackgroundColor(Color.rgb(7, 16, 25))
                settings.javaScriptEnabled = true
                settings.domStorageEnabled = true
                settings.cacheMode = WebSettings.LOAD_NO_CACHE
                settings.allowFileAccess = false
                settings.allowContentAccess = false
                settings.mediaPlaybackRequiresUserGesture = true
                settings.setSupportMultipleWindows(false)
                addJavascriptInterface(bridge, "Nav2Android")
                webChromeClient = WebChromeClient()
                webViewClient = WebViewClient()
                loadDataWithBaseURL(
                    endpoint.httpOrigin,
                    bundledPage,
                    "text/html",
                    "utf-8",
                    endpoint.httpOrigin,
                )
            }
        },
        modifier = modifier.fillMaxSize(),
        onRelease = { releasedView ->
            releasedView.removeJavascriptInterface("Nav2Android")
            releasedView.stopLoading()
            releasedView.loadUrl("about:blank")
            releasedView.destroy()
        },
    )
}
