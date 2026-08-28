机器狗 ARM64 Docker 离线部署说明
=================================

用途
----

这个目录用于 Nano 类 ARM64 移动端电脑。最终镜像包含：

  1. Ubuntu 22.04 + ROS 2 Humble ros-base
  2. Cartographer、cartographer_ros、cartographer_ros_msgs
  3. bondcpp 以及当前工作空间 package.xml 声明的全部外部依赖
  4. YDLidar SDK 1.2.20（固定到当前使用的提交）
  5. colcon、CMake 和单线程编译所需工具

镜像不预编译工作空间源码。Nav2、RF2O、YDLidar ROS 驱动和 Web 仍使用
cartographer_nav2_ws/src 中的代码，在机器人上执行一次单线程增量编译。


一、先确认机器人确实是 ARM64
------------------------------

在机器人宿主机执行：

  uname -m
  dpkg --print-architecture

必须分别得到 aarch64 和 arm64。若得到 x86_64/amd64，不要使用本镜像。


二、在有网电脑生成 ARM64 镜像 tar
---------------------------------

进入工作空间：

  cd /home/w/cartographer_nav2_ws

如果有网电脑本身就是 ARM64，直接执行：

  ./docker/arm64_offline/build_arm64_image.bash

如果有网电脑是 amd64/x86_64，需要先启用 QEMU 的 ARM64 binfmt。这个命令
需要联网拉取一次辅助镜像，并会在当前电脑注册 ARM64 模拟执行器。国内网络
默认通过 DaoCloud 的 Docker Hub 前缀拉取，再改成原镜像标签：

  docker pull m.daocloud.io/docker.io/tonistiigi/binfmt:latest
  docker tag m.daocloud.io/docker.io/tonistiigi/binfmt:latest \
    tonistiigi/binfmt:latest
  docker run --privileged --rm tonistiigi/binfmt:latest --install arm64

确认当前 builder 支持 ARM64：

  docker buildx ls

输出的 PLATFORMS 中应包含 linux/arm64。然后执行：

  ./docker/arm64_offline/build_arm64_image.bash

这一步会联网下载 ARM64 ROS/Ubuntu 软件包，可能耗时较长。它不会在当前
工作空间运行 colcon build。成功后工作空间根目录生成：

  dog-cartographer-nav2-humble-arm64.tar
  dog-cartographer-nav2-humble-arm64.tar.sha256

Docker Hub 镜像通过 DaoCloud 前缀访问；容器内 Ubuntu ARM、ROS 2 软件包和
rosdep 数据默认使用 USTC 镜像。YDLidar SDK 源码仍从 GitHub 下载，因此有网
电脑至少要能访问 GitHub、m.daocloud.io 和 mirrors.ustc.edu.cn。


三、把文件复制到离线机器人
--------------------------

需要复制：

  1. 整个 cartographer_nav2_ws，排除 build、install、log、bags
  2. dog-cartographer-nav2-humble-arm64.tar
  3. dog-cartographer-nav2-humble-arm64.tar.sha256

不要从 amd64 电脑复制 build、install 或 /usr/local/lib/libydlidar_sdk.a，
这些二进制文件不能在 ARM64 上运行。


四、机器人宿主机离线加载镜像
----------------------------

假设工作空间位于：

  /root/work/tracker/cartographer_nav2_ws

执行：

  cd /root/work/tracker/cartographer_nav2_ws
  ./docker/arm64_offline/load_image_on_robot.bash

该步骤只读取本地 tar，不访问网络。


五、启动容器并映射雷达
----------------------

确认真实设备：

  ls -l /dev/ttyUSB* /dev/serial/by-id/* 2>/dev/null

默认雷达为 /dev/ttyUSB0：

  ./docker/arm64_offline/run_robot_container.bash /dev/ttyUSB0

脚本使用 host 网络，Web 的 8081 端口和 ROS 2 DDS 不需要额外端口映射；
只把指定雷达设备传给容器，没有使用 --privileged。


六、容器内单线程编译
--------------------

  cd /root/work/tracker/cartographer_nav2_ws
  source /opt/ros/humble/setup.bash

  MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
    colcon build \
      --executor sequential \
      --symlink-install \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF

  source install/setup.bash

不要改回普通的 colcon build；普通命令会同时编译多个包，Nano 类设备容易
内存不足。之前 ydlidar_ros2_driver 的 unused parameter 是警告，可以忽略。


七、检查依赖和架构
------------------

  dpkg --print-architecture
  ros2 pkg prefix bondcpp
  ros2 pkg prefix cartographer_ros
  test -f /usr/local/lib/cmake/ydlidar_sdk/ydlidar_sdkConfig.cmake \
    && echo "YDLidar SDK OK"

预期：

  架构：arm64
  bondcpp：/opt/ros/humble
  cartographer_ros：/opt/ros/humble
  YDLidar SDK OK


八、启动建图
------------

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch dog_cartographer_nav2_bringup cartographer_mapping.launch.py

浏览器访问：

  http://机器人IP:8081

如果雷达不是 /dev/ttyUSB0，应在启动容器和 launch 参数中使用同一个实际设备。


九、开机后直接用手机控制 Launch（推荐）
----------------------------------------

手机不会直接执行宿主机命令。手机只连接 Web Bridge，由同一个 Docker
容器里的 Web Bridge 启动白名单中的建图或导航 launch。因此只需要让 Docker
和 Web Bridge 开机自启动，不要固定自启动某个建图或导航 launch。

先确保 Docker 服务随宿主机启动：

  sudo systemctl enable --now docker

工作空间完成编译后，只需创建一次后台容器：

  cd /root/work/tracker/cartographer_nav2_ws
  ./docker/arm64_offline/run_robot_web_service.bash /dev/ttyUSB0

该脚本使用 host 网络和 `--restart unless-stopped`，并在容器中自启动：

  ros2 launch dog_cartographer_nav2_bringup nav2_web_persistent.launch.py

之后手机访问：

  http://机器人IP:8081

在网页“手机 Launch 控制”中选择建图或导航。Web Bridge 停止子 launch 时，
网页和容器不会退出；宿主机重启后容器及 Web Bridge 会自动恢复，但不会自动
恢复上次选择的建图/导航流程，需要在手机上重新选择，避免无人值守时误运动。

常用维护命令：

  docker ps --filter name=dog_cartographer_nav2
  docker logs -f dog_cartographer_nav2
  docker restart dog_cartographer_nav2
  docker stop dog_cartographer_nav2
  docker start dog_cartographer_nav2

如果创建服务时没有连接雷达，脚本仍会启动 Web 页面，但该容器没有雷达设备
权限，不能进行真实建图。连接雷达后停止并删除该容器，再用正确设备重新创建。
