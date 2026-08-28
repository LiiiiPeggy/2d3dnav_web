/*
 * Copyright 2026 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 */

package com.dog.nav2controller

import kotlin.test.assertContains
import kotlin.test.assertFalse
import org.junit.Test

class Nav2WebViewTest {
    @Test
    fun bundlesVersionedAssetsWithoutInterpretingJavaScriptDollarSigns() {
        val script = """const url = `${'$'}{protocol}://${'$'}{host}`; const node = ${'$'}("map");"""
        val page = buildBundledNav2Page(
            html = """
                <html><head><link rel="stylesheet" href="style.css?v=3"></head>
                <body><script src="app.js?v=3"></script></body></html>
            """.trimIndent(),
            stylesheet = "body { color: white; }",
            script = script,
            webSocketPort = 8891,
        )

        assertContains(page, "body { color: white; }")
        assertContains(page, script)
        assertContains(page, "window.NAV2_ANDROID_WS_PORT = 8891")
        assertFalse("style.css?v=3" in page)
        assertFalse("app.js?v=3" in page)
    }
}
