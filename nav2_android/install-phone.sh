#!/usr/bin/env bash

set -euo pipefail

NAV2_ANDROID_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APK_PATH="$NAV2_ANDROID_DIR/nav2-app/build/outputs/apk/debug/nav2-app-debug.apk"

source "$NAV2_ANDROID_DIR/android-env.sh"

if [[ ! -f "$APK_PATH" ]]; then
  echo "APK does not exist; building it first..."
  "$NAV2_ANDROID_DIR/gradlew" -p "$NAV2_ANDROID_DIR" :nav2-app:assembleDebug
fi

adb start-server >/dev/null

AUTHORIZED_DEVICES="$(adb devices | awk 'NR > 1 && $2 == "device" { count++ } END { print count + 0 }')"
if [[ "$AUTHORIZED_DEVICES" -eq 0 ]]; then
  echo "No authorized Android phone found."
  echo "Enable USB debugging, reconnect the phone, and accept its RSA authorization dialog."
  adb devices -l
  exit 1
fi

adb install -r "$APK_PATH"
