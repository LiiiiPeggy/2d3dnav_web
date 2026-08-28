# 手机 Launch 控制：机器人 Docker 更新

## 要复制什么

机器人只需要以下 ROS/Web 后端文件：

- `src/web/nav2_web/`
- `src/dog_cartographer_nav2_bringup/`
- `docker/arm64_offline/run_robot_web_service.bash`

不要把电脑的 `build/`、`install/`、`log/` 复制到 ARM64 机器人；这些目录含有
当前电脑架构的构建结果。`nav2_android/` 也不需要放进机器人容器，APK 只安装
到手机。

本工作区生成的 `artifacts/nav2_robot_web_launch_0.2.0.tar.gz` 已包含上述机器人
文件。容器通过 volume 挂载工作空间，所以应把压缩包复制到机器人宿主机的
工作空间，而不是使用 `docker cp` 写入临时容器层。

## 从开发电脑复制

把 `ROBOT_IP` 换成机器人的地址：

```bash
scp /home/w/cartographer_nav2_ws/artifacts/nav2_robot_web_launch_0.2.0.tar.gz \
  root@ROBOT_IP:/root/work/tracker/cartographer_nav2_ws/
```

## 在机器人宿主机解压

```bash
cd /root/work/tracker/cartographer_nav2_ws
tar -xzf nav2_robot_web_launch_0.2.0.tar.gz
chmod 0755 docker/arm64_offline/run_robot_web_service.bash
```

解压只更新源代码和服务脚本，不覆盖 `maps/`。

## 已有后台容器时重新编译

先在手机停止当前受管建图或导航，再执行：

```bash
docker exec -it dog_cartographer_nav2 bash -c '
  cd /root/work/tracker/cartographer_nav2_ws &&
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1 colcon build \
    --executor sequential \
    --symlink-install \
    --packages-select nav2_web dog_cartographer_nav2_bringup \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
'

docker restart dog_cartographer_nav2
docker logs -f dog_cartographer_nav2
```

如果容器名称不是 `dog_cartographer_nav2`，先用 `docker ps --format
'{{.Names}}'` 查看实际名称并替换命令。

## 尚未创建后台容器时

先按照 `docker/arm64_offline/README_CN.txt` 在交互容器内完成 ARM64 编译，然后
创建一次开机自启动服务：

```bash
sudo systemctl enable --now docker
cd /root/work/tracker/cartographer_nav2_ws
./docker/arm64_offline/run_robot_web_service.bash /dev/ttyUSB0
```

后台容器只自动启动 `nav2_web_persistent.launch.py`。建图和导航仍由手机选择。

## 手机 APK

APK 不复制到机器人 Docker。开发电脑连接手机并开启 USB 调试后执行：

```bash
adb install -r \
  /home/w/cartographer_nav2_ws/nav2_android/nav2-app/build/outputs/apk/debug/nav2-app-debug.apk
```

安装后填写机器人宿主机 IP，HTTP 使用 `8081`，WebSocket 使用 `8891`。
