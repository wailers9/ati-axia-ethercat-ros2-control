# ATI Axia80-M20 EtherCAT ros2_control Sensor

本包提供一个 ROS 2 Jazzy `hardware_interface::SensorInterface` 插件，用于通过
IgH/EtherLab EtherCAT master 读取 ATI Axia80-M20 六维力/力矩传感器。

实现分为两层：

- ROS 2 Control 插件层：参考 `on-robot-ft-sensor` 的硬件接口组织方式。
- EtherCAT 驱动层：参考 `staman_robotiq_3f_gripper_lib-main` 的 IgH/EtherLab 调用方式。

## 项目结构

```text
ati_axia80_m20_ethercat_sensor/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── ati_axia80_m20_controllers.yaml
├── examples/
│   └── ati_axia80_m20_demo.urdf.xacro
├── include/ati_axia80_m20_ethercat_sensor/
│   ├── axia80_ethercat_config.hpp
│   ├── axia80_ethercat_driver.hpp
│   └── axia80_m20_ethercat_sensor.hpp
├── launch/
│   └── ati_axia80_m20_demo.launch.py
├── ros2_control/
│   ├── ati_axia80_m20_ethercat_sensor.xml
│   └── ati_axia80_m20_params.yaml
├── src/
│   ├── axia80_ethercat_config.cpp
│   ├── axia80_ethercat_driver.cpp
│   └── axia80_m20_ethercat_sensor.cpp
└── urdf/
    └── ati_axia80_m20_ethercat_sensor.xacro
```

## 当前能做什么

- 作为 ros2_control sensor hardware plugin 被 `controller_manager` 加载。
- 使用 IgH/EtherLab `ecrt.h` API 请求 EtherCAT master。
- 按 Vendor ID、Product Code、alias、position 查找 ATI Axia 从站。
- 配置 Axia EtherCAT PDO。
- 从 PDO 周期读取：
  - `Fx/Fy/Fz`
  - `Tx/Ty/Tz`
  - `status_code`
  - `sample_counter`
- 可选读取 SDO 标定比例：
  - `0x2021:0x37` counts per force
  - `0x2021:0x38` counts per torque
- 支持 `set_bias_on_activate` / `clear_bias_on_activate` 启动时 bias 控制。
- 提供手动 bias 服务：
  - `/ati_axia80_m20/set_bias`
  - `/ati_axia80_m20/clear_bias`
- 提供最小 demo launch，启动 `ros2_control_node` 和
  `force_torque_sensor_broadcaster`。

## 依赖

需要已安装：

- ROS 2 Jazzy
- ros2_control 相关包
- IgH/EtherLab EtherCAT master
- IgH/EtherLab 开发文件：`ecrt.h`、`libethercat.so`
- xacro

常用安装命令：

```bash
sudo apt-get install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-force-torque-sensor-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

IgH/EtherLab master 和开发库需要按本机环境单独安装。

## EtherCAT Master 配置

当前项目按以下网卡配置：

```bash
MASTER0_DEVICE="00:0c:29:ce:81:6d"
DEVICE_MODULES="generic"
```

配置文件通常是：

```bash
/etc/ethercat.conf
```

修改前应先备份：

```bash
sudo cp -a /etc/ethercat.conf /etc/ethercat.conf.bak.$(date +%Y%m%d-%H%M%S)
```

修改后启动服务：

```bash
sudo systemctl enable ethercat
sudo systemctl restart ethercat
```

检查 master：

```bash
grep -E '^(MASTER0_DEVICE|DEVICE_MODULES)=' /etc/ethercat.conf
systemctl status ethercat --no-pager
ethercat master
ethercat slaves
```

没有连接真实传感器时，`ethercat slaves` 没有输出或 `Slaves: 0` 是正常的。

## 构建

在 workspace 根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths ati_axia80_m20_ethercat_sensor --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

当前已验证：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

可以成功构建本包和参考包。

## 快速验证

验证 xacro 能正常展开：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
xacro install/ati_axia80_m20_ethercat_sensor/share/ati_axia80_m20_ethercat_sensor/examples/ati_axia80_m20_demo.urdf.xacro
```

验证 launch 参数：

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_demo.launch.py --show-args
```

## 运行 Demo

接上传感器后运行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_demo.launch.py \
  master_index:=0 \
  slave_position:=0 \
  slave_alias:=0
```

如果没有真实从站，启动到硬件激活阶段会报找不到 ATI Axia EtherCAT slave，
这是预期结果。

运行后可检查：

```bash
ros2 control list_hardware_interfaces
ros2 topic list
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

## ros2_control 插件

插件导出名：

```text
ati_axia80_m20_ethercat_sensor/Axia80M20EtherCATSensor
```

插件描述文件：

```text
ros2_control/ati_axia80_m20_ethercat_sensor.xml
```

可复用 xacro：

```text
urdf/ati_axia80_m20_ethercat_sensor.xacro
```

## State Interfaces

本插件导出以下状态接口：

```text
ati_axia80_m20/timestamp.sec
ati_axia80_m20/timestamp.nanosec
ati_axia80_m20/force.x
ati_axia80_m20/force.y
ati_axia80_m20/force.z
ati_axia80_m20/torque.x
ati_axia80_m20/torque.y
ati_axia80_m20/torque.z
ati_axia80_m20/status_code
ati_axia80_m20/sample_counter
```

`force_torque_sensor_broadcaster` 使用 `force.*` 和 `torque.*` 发布 wrench。

## 主要参数

这些参数在 `urdf/ati_axia80_m20_ethercat_sensor.xacro` 中配置。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `master_index` | `0` | IgH/EtherLab master index |
| `slave_position` | `0` | EtherCAT 从站位置 |
| `slave_alias` | `0` | EtherCAT alias，未配置 alias 时使用 0 |
| `vendor_id` | `0x00000732` | ATI Vendor ID |
| `product_code` | `0x26483053` | Axia EtherCAT Product Code |
| `revision` | `0x00000001` | Revision |
| `read_calibration_sdo` | `true` | 启动时读取 SDO 标定比例 |
| `counts_per_force` | `1000000` | 手动 force 缩放比例 |
| `counts_per_torque` | `1000000` | 手动 torque 缩放比例 |
| `filter_selection` | `0` | 传感器低通滤波选项，0..8 |
| `calibration_slot` | `0` | 标定槽位，0..1 |
| `sample_rate_code` | `0` | 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz |
| `clear_bias_on_activate` | `true` | 激活时清除已有 bias |
| `set_bias_on_activate` | `false` | 激活时设置 bias；默认关闭，避免传感器未稳定时设零点 |

如果现场 SDO 读取失败，可先把：

```xml
<param name="read_calibration_sdo">false</param>
```

然后手动填写 `counts_per_force` 和 `counts_per_torque`。
这两个比例必须是有限正数，不能为 0。

## 手动 Bias 控制

硬件激活后，可以用 `std_srvs/srv/Trigger` 服务手动归零或取消归零：

```bash
ros2 service call /ati_axia80_m20/set_bias std_srvs/srv/Trigger '{}'
ros2 service call /ati_axia80_m20/clear_bias std_srvs/srv/Trigger '{}'
```

建议流程：

1. 启动 demo，等待传感器稳定且无外载。
2. 调用 `/ati_axia80_m20/set_bias` 手动归零。
3. 如果需要恢复未 bias 的原始输出，调用 `/ati_axia80_m20/clear_bias`。

如果驱动尚未配置或硬件未 active，服务会返回 `success=false` 和错误信息。

## PDO 映射

PDO 布局来自 `200807 - ATI Axia EtherCAT FT.xml`。

RxPDO `0x1601`：

```text
0x7010:01 Control 1
0x7010:02 Control 2
```

TxPDO `0x1A00`：

```text
0x6000:01 Fx
0x6000:02 Fy
0x6000:03 Fz
0x6000:04 Tx
0x6000:05 Ty
0x6000:06 Tz
0x6010:00 Status code
0x6020:00 Sample counter
```

## 排障

检查 EtherCAT 服务：

```bash
systemctl status ethercat --no-pager
journalctl -u ethercat --no-pager -n 80
```

检查 master 和从站：

```bash
ethercat master
ethercat slaves
```

检查内核模块：

```bash
lsmod | grep -i ethercat
dmesg | grep -i ethercat | tail -50
```

如果 `dmesg` 显示权限不足，需要在本机终端使用 `sudo dmesg`。

常见情况：

- `Slaves: 0`：未连接传感器、线缆/供电问题、网卡不是实际 EtherCAT 网口。
- `Frame loss 100%`：没有从站回复时正常；接上传感器后应下降。
- `failed to request EtherCAT master`：服务未启动，或当前用户没有访问权限。
- `ATI Axia EtherCAT slave was not found`：master 工作但未扫描到匹配 Vendor/Product/position。
- SDO 读取失败：先设置 `read_calibration_sdo=false`，使用手动缩放比例验证 PDO 通讯。

## 当前状态

- EtherCAT master 已配置到 `00:0c:29:ce:81:6d`。
- `ethercat.service` 已能启动。
- 未连接真实 ATI Axia80-M20 时，从站数量为 0 是正常状态。
- 包已通过 ROS 2 Jazzy 构建。
- `xacro` 已安装并验证 demo URDF 可展开。
