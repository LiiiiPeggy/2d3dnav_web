#!/usr/bin/env bash

NAV2_ANDROID_PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV2_ANDROID_LOCAL_SDK="$(
  sed -n 's/^sdk\.dir=//p' "$NAV2_ANDROID_PROJECT_ROOT/local.properties" 2>/dev/null \
    | head -n 1
)"

if [[ -z "$NAV2_ANDROID_LOCAL_SDK" || ! -d "$NAV2_ANDROID_LOCAL_SDK" ]]; then
  NAV2_ANDROID_LOCAL_SDK="/home/w/cartographer_nav2_ws/tools/android-sdk"
fi
NAV2_ANDROID_LOCAL_JDK="$(dirname "$NAV2_ANDROID_LOCAL_SDK")/android-studio/jbr"

if [[ ! -x "$NAV2_ANDROID_LOCAL_JDK/bin/java" ]]; then
  echo "Android JDK 不存在: $NAV2_ANDROID_LOCAL_JDK" >&2
  return 1 2>/dev/null || exit 1
fi

export JAVA_HOME="$NAV2_ANDROID_LOCAL_JDK"
export ANDROID_HOME="$NAV2_ANDROID_LOCAL_SDK"
export ANDROID_SDK_ROOT="$ANDROID_HOME"
export PATH="$JAVA_HOME/bin:$ANDROID_HOME/platform-tools:$ANDROID_HOME/emulator:$ANDROID_HOME/cmdline-tools/latest/bin:$PATH"
