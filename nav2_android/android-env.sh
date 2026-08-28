#!/usr/bin/env bash

NAV2_ANDROID_WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export JAVA_HOME="$NAV2_ANDROID_WORKSPACE_ROOT/tools/android-studio/jbr"
export ANDROID_HOME="$NAV2_ANDROID_WORKSPACE_ROOT/tools/android-sdk"
export ANDROID_SDK_ROOT="$ANDROID_HOME"
export PATH="$JAVA_HOME/bin:$ANDROID_HOME/platform-tools:$ANDROID_HOME/emulator:$ANDROID_HOME/cmdline-tools/latest/bin:$PATH"
