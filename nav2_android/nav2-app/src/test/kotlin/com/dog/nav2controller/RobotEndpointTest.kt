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

import kotlin.test.assertEquals
import kotlin.test.assertTrue
import org.junit.Test

class RobotEndpointTest {
    @Test
    fun plainIpv4UsesDefaultPorts() {
        val endpoint = RobotEndpoint.parse("10.10.10.186", "", "").getOrThrow()

        assertEquals("10.10.10.186", endpoint.host)
        assertEquals(8081, endpoint.httpPort)
        assertEquals(8891, endpoint.webSocketPort)
        assertEquals("http://10.10.10.186:8081/", endpoint.httpOrigin)
    }

    @Test
    fun pastedHttpAddressCanSupplyHttpPort() {
        val endpoint = RobotEndpoint.parse("http://robot.local:9080/", "", "9900")
            .getOrThrow()

        assertEquals("robot.local", endpoint.host)
        assertEquals(9080, endpoint.httpPort)
        assertEquals(9900, endpoint.webSocketPort)
    }

    @Test
    fun pathAndInvalidPortAreRejected() {
        assertTrue(RobotEndpoint.parse("robot.local/nav", "8081", "8891").isFailure)
        assertTrue(RobotEndpoint.parse("robot.local", "0", "8891").isFailure)
        assertTrue(RobotEndpoint.parse("robot.local", "not-a-port", "8891").isFailure)
        assertTrue(RobotEndpoint.parse("ftp://robot.local", "8081", "8891").isFailure)
    }
}
