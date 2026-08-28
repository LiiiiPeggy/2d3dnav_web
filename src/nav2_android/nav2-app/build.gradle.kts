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

plugins {
    alias(libs.plugins.nowinandroid.android.application)
    alias(libs.plugins.nowinandroid.android.application.compose)
}

android {
    namespace = "com.dog.nav2controller"

    defaultConfig {
        // Keep planner independent from the existing Dog Nav2 installation.
        applicationId = "com.dog.planner"
        versionCode = 25
        versionName = "0.6.2"
    }

    buildTypes {
        debug {
            applicationIdSuffix = ".debug"
        }
        release {
            isMinifyEnabled = true
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro",
            )
            signingConfig = signingConfigs.named("debug").get()
        }
    }
}

dependencies {
    implementation(libs.androidx.activity.compose)
    implementation(libs.androidx.compose.foundation)
    implementation(libs.androidx.compose.material3)
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.lifecycle.runtimeCompose)
    implementation(libs.androidx.lifecycle.viewModelCompose)

    testImplementation(libs.junit)
    testImplementation(libs.kotlin.test)
}

dependencyGuard {
    configuration("releaseRuntimeClasspath")
}

val copyPlannerDebugApk by tasks.registering(org.gradle.api.tasks.Copy::class) {
    from(layout.buildDirectory.file("outputs/apk/debug/nav2-app-debug.apk"))
    into(layout.buildDirectory.dir("outputs/planner"))
    rename { "planner.apk" }
}

tasks.matching { it.name == "assembleDebug" }.configureEach {
    finalizedBy(copyPlannerDebugApk)
}
