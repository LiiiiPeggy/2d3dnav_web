# Dog Nav2 Android App

`nav2-app` is the robot-control application built on the Now in Android Kotlin and Jetpack Compose
project. The upstream sample modules remain available as architecture references; the runnable robot
application is the separate `:nav2-app` module.

## Why the UI survives ROS launch changes

The HTML, CSS, and JavaScript from `src/web/nav2_web/web` are packaged in the APK. A WebView loads
those files from the APK while keeping the page origin set to the configured robot address. Only the
WebSocket data connection goes to the robot. Consequently:

- stopping Cartographer or Nav2 does not unload the Android UI;
- stopping and restarting the Web Bridge changes the status to disconnected and the existing
  JavaScript reconnect loop reconnects automatically;
- reopening the App while the robot is offline still shows the local connection screen;
- the last robot host and ports are retained in Android preferences.

The App uses the existing `nav2_web` JSON protocol, so map rendering, laser/particle overlays,
costmaps, navigation goals, initial pose, map saving, localization reset, and inflation parameter
editing retain the behavior of the current browser UI.

## Touch and responsive layout

- drag the map with one finger;
- pinch with two fingers, or use the `+` / `-` buttons, to zoom;
- after selecting a navigation goal or initial pose, press the map, drag to choose the heading, and
  release to send it;
- swipe the status panel vertically to reach the cards below it;
- portrait phones use a map-above-panel layout, while landscape phones and tablets use two columns.
- map buttons collapse to icon-only/two-column toolbars on narrow screens, and the map automatically
  refits after rotation, split-screen resizing, or a browser viewport-height change;
- drag the divider between the map and control panel to resize the main windows; the 设置 tab can
  save separate portrait/landscape ratios and the preferred map-control size on the current device.
- in landscape, the bottom position/navigation summary can be shown or hidden, resized vertically,
  and split horizontally between its two cards; these values are saved with the rest of the layout.

## Build

Requirements follow the checked-out Now in Android revision:

- JDK 17 or newer;
- Android SDK 36;
- a recent stable Android Studio.

Open this directory in Android Studio and run the `nav2-app` configuration, or build from a shell:

```bash
source ./android-env.sh
./gradlew :nav2-app:assembleDebug
```

The APK is generated under `nav2-app/build/outputs/apk/debug/`.

## Install on an Android phone

Enable Developer options and USB debugging on the phone, connect it over USB, and accept the RSA
authorization dialog shown on the phone. Then run:

```bash
source ./android-env.sh
adb devices -l
./install-phone.sh
```

The installed application is named **Dog Nav2**. The debug build supports Android 6.0 (API 23) and
newer. Re-running `adb install -r` upgrades the existing debug installation while keeping its saved
robot endpoint.

## Robot-side startup

Build and source the ROS workspace, then start the Web Bridge once in its own terminal or service:

```bash
source /opt/ros/humble/setup.bash
source /home/w/cartographer_nav2_ws/install/setup.bash
ros2 launch dog_cartographer_nav2_bringup nav2_web_persistent.launch.py
```

This command is the persistent "launch starting point". In a phone browser, open
`http://ROBOT_IP:8081`; the **建图** and **导航** tabs expose only their corresponding allowlisted
profiles and display their live logs. Only one Web-managed profile can run at a time. If a mapping
or navigation launch is already running in another terminal, stop it before starting the same stack
from the Web page.

App version 0.2.6 uses the same four workflow tabs as the browser UI: 建图、导航、状态、设置.
The mapping and navigation pages independently select the relevant launch profile, while sharing
the same single-process start/stop safety rule, process state, and live Launch logs.

The browser UI is the canonical source under `src/web/nav2_web/web/`. After changing its
`index.html`, `style.css`, or `app.js`, synchronize the exact same UI into the APK assets before
building:

```bash
./nav2_android/sync-nav2-ui.sh
```

The command-line equivalents remain available as a fallback:

```bash
ros2 launch dog_cartographer_nav2_bringup cartographer_mapping.launch.py
# Stop mapping when finished, then for example:
ros2 launch dog_cartographer_nav2_bringup nav2_mppi_navigation.launch.py
```

Their `start_web` argument now defaults to `False`. Do not pass `start_web:=True` while the
persistent bridge is active, because both processes would try to bind HTTP port 8081 and WebSocket
port 8891.

On first Android launch, enter the robot IP or hostname. The defaults are HTTP `8081` and WebSocket
`8891`.

When ROS runs in the ARM64 Docker image, the phone does not execute commands on the host. The Web
Bridge runs the allowlisted `ros2 launch` command inside its own container, so the phone does not
need to know a workspace directory. The package only needs to be built and sourced in that
container.

For boot persistence, enable Docker and create the background Web-control container once:

```bash
sudo systemctl enable --now docker
cd /root/work/tracker/cartographer_nav2_ws
./docker/arm64_offline/run_robot_web_service.bash /dev/ttyUSB0
```

The container uses `--restart unless-stopped` and automatically starts only
`nav2_web_persistent.launch.py`. Mapping and navigation are deliberately selected from the phone
after boot. This keeps the Web UI alive across child launch changes and avoids unattended robot
motion.

For the exact robot-side files, ARM64 rebuild commands, and APK installation command, see
[`ROBOT_DOCKER_DEPLOY.md`](../ROBOT_DOCKER_DEPLOY.md).

## Network note

The current bridge uses unencrypted local-network `http://` and `ws://` without authentication.
The Android manifest therefore permits cleartext traffic. Use it only on a trusted robot network;
authentication and TLS should be added before exposing the bridge beyond that network.
