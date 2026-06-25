# ATI Axia80-M20 EtherCAT ros2_control Sensor

这是一个 ROS 2 Jazzy `ros2_control` 传感器插件，用于通过 IgH/EtherLab
EtherCAT master 读取 ATI Axia80-M20 六维力/力矩传感器，并通过
`force_torque_sensor_broadcaster` 发布 `geometry_msgs/msg/WrenchStamped` topic。

## 功能

- 作为 `hardware_interface::SensorInterface` 被 `controller_manager` 加载。
- 通过 IgH/EtherLab `ecrt.h` API 与 EtherCAT master 通讯。
- 按 Vendor ID、Product Code、alias、position 查找 ATI Axia 从站。
- 配置 Axia EtherCAT PDO 并读取：
  - `Fx/Fy/Fz`
  - `Tx/Ty/Tz`
  - `status_code`
  - `sample_counter`
- `read()` 中保留 sample counter 检查，仅累计统计量，每秒通过 `/diagnostics`
  发布，不再逐次打印告警。
- 实时读取和低频 diagnostics 之间通过同步保护的轻量快照传递状态。
- diagnostics 分为 communication、sample counter、runtime SDO 和 sensor status
  四个独立状态项。
- 可选从 SDO `0x2021:0x37` / `0x2021:0x38` 读取 force/torque 缩放比例。
- 防止 `counts_per_force` 和 `counts_per_torque` 为 0 或无效值导致除零。
- 提供手动 bias 服务：
  - `/ati_axia80_m20/set_bias`
  - `/ati_axia80_m20/clear_bias`

## 依赖

需要先安装：

- ROS 2 Jazzy
- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `force_torque_sensor_broadcaster`
- `robot_state_publisher`
- `xacro`
- IgH/EtherLab EtherCAT master
- IgH/EtherLab 开发文件：`ecrt.h`、`libethercat.so`

ROS 依赖安装示例：

```bash
sudo apt-get install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-force-torque-sensor-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

EtherCAT master 和开发库的安装方式取决于你的系统环境。确认 `ecrt.h` 和
`libethercat.so` 能被 CMake 找到后再构建本包。

## 自动化测试

无需真实硬件的 GoogleTest 覆盖参数解析、sample counter 统计、独立 diagnostics
level、mock driver bias 行为和纯函数：

```bash
colcon test --packages-select ati_axia80_m20_ethercat_sensor
```

## 克隆和构建

在 ROS 2 workspace 的 `src` 目录中克隆：

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/wailers9/ati-axia-ethercat-ros2-control.git
```

构建：

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ati_axia80_m20_ethercat_sensor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## EtherCAT 准备

需要先把 IgH/EtherLab master 绑定到本机连接 ATI 传感器的 EtherCAT 网口。

需要确认和填写的信息：

| 信息 | 填写位置 | 说明 |
| --- | --- | --- |
| EtherCAT 网卡 MAC 地址 | `/etc/ethercat.conf` 的 `MASTER0_DEVICE` | 使用本机连接传感器的有线网口，不要使用 Wi-Fi、外网网口或别人机器的 MAC |
| EtherCAT 驱动模块 | `/etc/ethercat.conf` 的 `DEVICE_MODULES` | 常见值是 `generic`，如使用专用驱动请按你的 EtherCAT master 安装说明填写 |
| master index | launch 参数 `master_index` / xacro 参数 `master_index` | 第一个 master 通常是 `0` |
| slave position | launch 参数 `slave_position` / xacro 参数 `slave_position` | ATI 传感器在 EtherCAT 总线中的从站位置，通常单个从站时是 `0` |
| slave alias | launch 参数 `slave_alias` / xacro 参数 `slave_alias` | 未配置 EtherCAT alias 时使用 `0` |

查找本机网卡：

```bash
ip link
```

找到连接 ATI 传感器的有线网卡名，例如 `enp3s0`。查看该网卡 MAC 地址：

```bash
ip link show enp3s0
```

输出中的 `link/ether` 后面就是要填入 `MASTER0_DEVICE` 的 MAC 地址。

编辑 EtherCAT master 配置：

```bash
sudo nano /etc/ethercat.conf
```

配置格式示例：

```bash
MASTER0_DEVICE="<你的 EtherCAT 网卡 MAC 地址>"
DEVICE_MODULES="generic"
```

保存后重启 EtherCAT 服务：

```bash
sudo systemctl restart ethercat
```

常用检查命令：

```bash
systemctl status ethercat --no-pager
ethercat master
ethercat slaves
```

连接传感器并启动 EtherCAT master 后，`ethercat slaves` 应能看到 ATI Axia
从站。如果没有从站，ROS 2 硬件激活会失败，后续不会发布有效力数据。

如果有多个 EtherCAT 从站，用下面的命令确认传感器位置：

```bash
ethercat slaves
```

输出列表中的顺序位置就是 `slave_position`。例如传感器是第一个从站，通常填
`slave_position:=0`。

## 使用插件

插件导出名：

```text
ati_axia80_m20_ethercat_sensor/Axia80M20EtherCATSensor
```

可直接复用：

```text
ati_axia80_m20_ethercat_sensor/urdf/ati_axia80_m20_ethercat_sensor.xacro
```

demo URDF 位于：

```text
ati_axia80_m20_ethercat_sensor/examples/ati_axia80_m20_demo.urdf.xacro
```

启动 demo：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_demo.launch.py \
  master_index:=0 \
  slave_position:=0 \
  slave_alias:=0
```

参数说明：

- `master_index`：IgH/EtherLab master index，通常从 `0` 开始。
- `slave_position`：传感器在 EtherCAT 总线中的位置。
- `slave_alias`：EtherCAT alias，未配置 alias 时使用 `0`。

这三个参数和 `/etc/ethercat.conf` 的关系：

- `/etc/ethercat.conf` 决定 IgH/EtherLab master 使用哪块本机网卡。
- `master_index` 决定 ROS 2 插件请求哪个 IgH/EtherLab master。
- `slave_position` / `slave_alias` 决定插件在该 master 下连接哪个 ATI Axia 从站。

## 获取力/力矩 Topic

demo launch 会启动：

- `ros2_control_node`
- `robot_state_publisher`
- `force_torque_sensor_broadcaster`

力/力矩 topic：

```text
/ati_axia80_m20_broadcaster/wrench
```

查看 topic：

```bash
ros2 topic list
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

检查硬件接口和 controller 状态：

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

`force_torque_sensor_broadcaster` 会读取以下 state interfaces 并发布 wrench：

```text
ati_axia80_m20/force.x
ati_axia80_m20/force.y
ati_axia80_m20/force.z
ati_axia80_m20/torque.x
ati_axia80_m20/torque.y
ati_axia80_m20/torque.z
```

插件还导出：

```text
ati_axia80_m20/timestamp.sec
ati_axia80_m20/timestamp.nanosec
ati_axia80_m20/status_code
ati_axia80_m20/sample_counter
```

## 手动归零和取消归零

传感器硬件 active 后，可以使用 `std_srvs/srv/Trigger` 服务手动控制 bias。

手动归零：

```bash
ros2 service call /ati_axia80_m20/set_bias std_srvs/srv/Trigger '{}'
```

取消归零：

```bash
ros2 service call /ati_axia80_m20/clear_bias std_srvs/srv/Trigger '{}'
```

建议流程：

1. 启动系统并等待传感器稳定。
2. 确认传感器处于期望的零点受力状态。
3. 调用 `/ati_axia80_m20/set_bias`。
4. 用 `/ati_axia80_m20_broadcaster/wrench` 确认输出。
5. 如需恢复未 bias 的输出，调用 `/ati_axia80_m20/clear_bias`。

如果驱动尚未配置或硬件未 active，服务会返回 `success=false` 和错误信息。

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
| `read_calibration_sdo` | `true` | 激活时读取 SDO 标定比例 |
| `runtime_diagnostic_sdo` | `true` | 在 `/diagnostics` 中读取运行时 SDO 诊断 |
| `runtime_diagnostic_sdo_timeout_ms` | `5` | 运行时 SDO 诊断自动暂停阈值，单位 ms |
| `counts_per_force` | `1000000` | 手动 force 缩放比例 |
| `counts_per_torque` | `1000000` | 手动 torque 缩放比例 |
| `filter_selection` | `0` | 传感器低通滤波选项，0..8 |
| `calibration_slot` | `0` | 标定槽位，0..1 |
| `sample_rate_code` | `1` | 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz |
| `expected_sensor_rate_hz` | `975` | sample counter 诊断使用的传感器频率 |
| `expected_read_rate_hz` | `975` | 诊断使用的 ROS read 频率，应与 controller manager 一致 |
| `clear_bias_on_activate` | `true` | 激活时清除已有 bias |
| `set_bias_on_activate` | `false` | 激活时自动设置 bias，默认关闭 |

默认不启用 `set_bias_on_activate`，避免传感器刚启动未稳定时自动设置零点。

如果现场 SDO 读取失败，可设置：

```xml
<param name="read_calibration_sdo">false</param>
```

然后手动填写 `counts_per_force` 和 `counts_per_torque`。这两个值必须是有限正数，
不能为 0。
启动阶段 calibration SDO 发生在实时 read loop 之前；如果激活阶段 SDO 耗时超过
500 ms，驱动会打印告警。

## Web 监测系统

保留原有 demo launch 的同时，可以用 full launch 一键启动传感器、wrench 发布和监测网页：

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py
```

从 GitHub 拉取项目到新机器后，推荐按下面顺序安装额外依赖、构建、source 工作区并启动：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rclpy \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-controller-manager-msgs \
  python3-aiohttp

cd ~/starman/ati-axia-ethercat-ros2-control
colcon build --packages-select ati_axia80_m20_ethercat_sensor
source install/setup.bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py
```

监测节点不直接控制 EtherCAT，也不参与实时控制路径。service、topic、hardware interface
等查询默认每 10 秒检查一次，也就是 0.1 Hz。完整网页监测数据，例如 diagnostics、电压、
温度、EtherCAT health 和 sample counter 统计，默认以 1 Hz 推送到网页。六维力 wrench 数据默认以 25 Hz
通过 WebSocket 推送到网页，网页六张图表默认以 20 Hz 重绘。启动后终端会打印一次网页监听地址，
默认端口为 `8765`：

```text
Axia80 monitor dashboard listening on http://0.0.0.0:8765/
```

本机访问：

```text
http://127.0.0.1:8765/
```

局域网其它设备访问：

```text
http://<传感器电脑IP>:8765/
```

可通过参数修改监听地址、端口和低频检查周期：

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py \
  monitor_host:=0.0.0.0 \
  monitor_port:=8765 \
  monitor_check_period_sec:=10.0 \
  monitor_telemetry_period_sec:=1.0 \
  monitor_wrench_push_rate_hz:=25.0 \
  monitor_chart_refresh_rate_hz:=20.0
```

网页各部分内容：

- `System Overview`：显示整体报警状态、wrench 数据流状态、EtherCAT 状态、从诊断信息提取的温度和电压。
- `EtherCAT Health`：显示后端根据驱动自身 EtherCAT 诊断和状态值判断出的检查结果，
  包括 master link、响应从站数、Axia slave online/operational、status code、
  runtime diagnostic SDO 状态等。这些值来自驱动已经检查并打印到终端的同类状态，
  同时通过 `/diagnostics` 发布给监测节点。
- `Manual Bias Control`：提供 `Set Bias` 和 `Clear Bias` 按钮。网页会先弹窗确认，
  再调用 `/ati_axia80_m20/set_bias` 或 `/ati_axia80_m20/clear_bias`。
- `Force / Torque Channels`：六维力/力矩分开显示，Fx、Fy、Fz、Tx、Ty、Tz 各自一张动态图表，
  并显示每个通道最新值。wrench WebSocket 推送建议 20-50 Hz，网页图表刷新建议 10-30 Hz。
- `ROS 2 Checks`：检查 service 类型、topic 类型、wrench 是否持续更新、force/torque 是否为有限数值、
  预期 hardware state interface 是否可用，以及低频 EtherCAT health 汇总。
- `Hardware Interfaces`：显示 `/controller_manager/list_hardware_interfaces` 返回的 state interfaces，
  包括接口名、是否 available、是否 claimed。
- `Diagnostics`：显示 `/diagnostics` 表格。包含 `temperature`、`temp`、`voltage` 或 `volt`
  的诊断字段会同时提升到顶部 Overview 指标。

网页中的 `Set Bias` 和 `Clear Bias` 按钮会先弹窗确认，再调用对应 ROS service。
调用完成后监测节点会继续检查 wrench 是否更新、force/torque 是否仍为有效数值。

## 在自己的机器人描述中使用

在你的机器人 xacro 中 include 本包的 xacro：

```xml
<xacro:include filename="$(find ati_axia80_m20_ethercat_sensor)/urdf/ati_axia80_m20_ethercat_sensor.xacro"/>

<xacro:ati_axia80_m20_ethercat_sensor
  name="ati_axia80_m20"
  parent="tool0"
  master_index="0"
  slave_position="0"
  slave_alias="0"/>
```

如果你的末端 link 不是 `tool0`，请把 `parent` 改成实际 link 名称。

## 诊断和调试

ATI 建议监控 `0x6010: Status Code`，而不是标准 EtherCAT
`0x1001 Error Register`。驱动内部保留原始 `uint32_t` status word，将它导出为
ROS 2 control `double` state interface，并在 status word 变化时打印解析后的 bit
说明。解析后的 status bit 摘要也会通过 ROS diagnostics 的 `status_bits` 字段发布。

controller active 后可用下面的命令检查：

```bash
ros2 topic echo /diagnostics
ros2 control list_hardware_interfaces
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

`/diagnostics` 中应能看到这些 key：

```text
status_code
status_bits
sample_counter
sdo_success
sdo_skipped
sdo_failed
runtime_sdo_last_elapsed_us
runtime_sdo_consecutive_lock_failures
runtime_sdo_auto_paused
runtime_sdo_pause_reason
supply_voltage_v
gage_temperature_c
diagnostic_status_message
expected_repeats_per_sec
actual_repeats_per_sec
repeat_rate
expected_skipped_samples_per_sec
expected_jump_events_per_sec
actual_skipped_samples_per_sec
actual_jump_events_per_sec
skipped_samples
max_delta
consecutive_repeats
sample_counter_status
```

高频 `read()` 周期每次都会执行 PDO receive/process 和 PDO queue/send。每个有效
PDO sample 都会更新 `status_code`、解析 status bits，并检查 `sample_counter`
，但实时循环只累计数据。诊断每秒评估一次：连续重复 10/50 次分别为 WARN/ERROR；
频率阈值使用理论值 2 倍再加固定余量（重复 60/100，跳样 30/50）。
是否重复或跳变。

EtherCAT master、domain、slave state 检查被限制为每 1 秒最多执行一次，并且只在
状态变化时打印日志。激活后的第一次 `read_once()` 一定会执行一次 EtherCAT state
检查。

`runtime_diagnostic_sdo` 为 true 时，`/diagnostics` publisher 以 1 Hz 在实时
`read()` 路径之外读取 `0x2080: Diagnostic Readings` SDO。运行时 SDO 使用
`try_lock`，不会等待 EtherCAT driver 锁；目标耗时小于 500 us。单次超过 1 ms
会打印告警，单次超过 `runtime_diagnostic_sdo_timeout_ms` 或连续 5 次抢不到 driver
锁时，会自动暂停运行时 SDO。自动暂停后 `/diagnostics` 会显示
`runtime_sdo_auto_paused=true` 和 `runtime_sdo_pause_reason`。如果某次 SDO 到下一次
diagnostics tick 仍未结束，驱动不会并发启动第二个运行时 SDO。它会报告外部供电电压、
应变片温度和 ATI priority status message，包括电压故障、温度故障、标定校验错误、
应变片断开或超限、力/力矩量程超限、硬件或协议栈错误、模拟错误以及未分类错误。

如果 `sample_counter` 跳变严重，或 `/diagnostics` 持续显示运行时 SDO 因为
EtherCAT driver busy 而抢不到锁，应关闭运行时 SDO 诊断，只依赖 PDO 数据和状态日志：

```xml
<param name="runtime_diagnostic_sdo">false</param>
```

这也是手动暂停运行时 SDO 的指令。如需重新启用，在启动前把
`runtime_diagnostic_sdo` 改回 `true`。

## PDO 映射

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

## 常见问题

- `ATI Axia EtherCAT slave was not found`：检查 EtherCAT master、线缆、供电、
  `slave_position`、Vendor ID 和 Product Code。
- 没有 `/ati_axia80_m20_broadcaster/wrench`：检查 controller 是否 loaded/active。
- topic 存在但没有有效数据：检查硬件是否 active，以及 `ethercat slaves` 是否能看到传感器。
- SDO 读取失败：将 `read_calibration_sdo` 设为 `false`，手动填写
  `counts_per_force` 和 `counts_per_torque` 后再测试 PDO 通讯。
- `failed to request EtherCAT master`：检查 EtherCAT 服务是否启动，以及当前用户是否有权限访问 master。
