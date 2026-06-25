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
- `read()` 中保留 `sample_counter` 检查，但不再逐次打印重复/跳变告警；统计结果每秒
  发布到 `/diagnostics`。
- 实时读取路径只更新一个带同步保护的轻量诊断快照，1 Hz diagnostics 只读取该快照，
  不再并发读取正在变化的 PDO/driver 状态。
- `/diagnostics` 拆分为四个独立状态项：
  - `Axia80 EtherCAT communication`
  - `Axia80 sample counter`
  - `Axia80 runtime SDO diagnostics`
  - `Axia80 sensor status code`
  因此 runtime SDO 的 `STALE` 不会覆盖 sample counter 的 `WARN` 或 `ERROR`。
- `status_code` 快速抖动以及 `read()` 失败日志均带 throttle，避免高频刷屏。
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

项目包含无需真实硬件的 GoogleTest，覆盖参数校验、sample counter 统计与回绕、
独立 diagnostics level、mock driver 下的 bias 状态，以及 status bit、hex 和 bool
等纯函数。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ati_axia80_m20_ethercat_sensor
colcon test --packages-select ati_axia80_m20_ethercat_sensor --event-handlers console_direct+
colcon test-result --verbose
```

这些连接真实硬件前的测试不会请求 EtherCAT master，也不需要连接传感器。

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
| `expected_sensor_rate_hz` | `975` | sample counter 诊断使用的传感器频率；未配置时按 `sample_rate_code` 精确频率推导 |
| `expected_read_rate_hz` | `975` | 诊断使用的 ROS `read()` 频率，应与 `controller_manager.update_rate` 保持一致 |
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
expected_sensor_rate_hz
expected_read_rate_hz
expected_repeats_per_sec
actual_repeats_per_sec
repeat_rate
expected_skipped_samples_per_sec
expected_jump_events_per_sec
actual_skipped_samples_per_sec
actual_jump_events_per_sec
repeated_reads
skipped_samples
jump_events
max_delta
large_jump_threshold
consecutive_repeats
max_consecutive_repeats
sample_counter_status
```

高频 `read()` 周期每次都会执行 PDO receive/process 和 PDO queue/send。每个有效
PDO sample 都会更新 `status_code`、解析 status bits，并检查 `sample_counter`
是否重复或跳变。检查函数只累计统计量，不在实时循环中直接报警。

默认 `controller_manager.update_rate=975 Hz`，与手册中
`sample_rate_code=1` 的 1 kHz 档精确采样率一致。可在启动时同时修改：

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py \
  sample_rate_code:=1 expected_sensor_rate_hz:=975 read_rate_hz:=975
```

sample counter 诊断每 1 秒评估一次：连续重复 10 次为 `WARN`，连续 50 次为
`ERROR`。重复率和跳样率使用 `2 * 理论值 + 固定余量`：重复每秒余量
`60/100`，跳样每秒余量 `30/50`，分别对应 WARN/ERROR。单次 delta 超过
`max(10, ceil(5 * sensor_rate / read_rate))` 为 WARN，超过该阈值 5 倍为 ERROR。

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
