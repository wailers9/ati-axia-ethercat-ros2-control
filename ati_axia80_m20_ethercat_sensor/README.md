# ATI Axia80-M20 EtherCAT ros2_control Sensor

This package provides a ROS 2 Jazzy `ros2_control` sensor plugin for reading an
ATI Axia80-M20 six-axis force/torque sensor through an IgH/EtherLab EtherCAT
master. It publishes `geometry_msgs/msg/WrenchStamped` data through
`force_torque_sensor_broadcaster`.

[Chinese documentation](README.zh-CN.md)

## Features

- Loads as a `hardware_interface::SensorInterface` through `controller_manager`.
- Communicates with the EtherCAT master through the IgH/EtherLab `ecrt.h` API.
- Finds the ATI Axia slave by Vendor ID, Product Code, alias, and position.
- Configures the Axia EtherCAT PDOs and reads:
  - `Fx/Fy/Fz`
  - `Tx/Ty/Tz`
  - `status_code`
  - `sample_counter`
- Decodes ATI `0x6010` status bits and throttles rapidly changing status logs.
- Keeps `sample_counter` checks in `read()` and publishes one-second
  repeat/jump statistics through `/diagnostics` without per-sample warnings.
- Optionally reads force/torque scale factors from SDO `0x2021:0x37` /
  `0x2021:0x38`, plus serial number, calibration part number, calibration time,
  and active calibration slot.
- Publishes ROS 2 diagnostics from low-rate SDO reads of `0x2080`:
  supply voltage, gage temperature, and ATI priority status message.
- Logs EtherCAT master, domain, working counter, and slave state changes.
- Guards against invalid or zero `counts_per_force` and `counts_per_torque`
  values to avoid division by zero.
- Provides manual bias services:
  - `/<sensor_name>/set_bias`
  - `/<sensor_name>/clear_bias`

## Dependencies

Install these first:

- ROS 2 Jazzy
- `ros2_control`
- `ros2_controllers`
- `controller_manager`
- `force_torque_sensor_broadcaster`
- `robot_state_publisher`
- `xacro`
- IgH/EtherLab EtherCAT master
- IgH/EtherLab development files: `ecrt.h`, `libethercat.so`

Example ROS dependency installation:

```bash
sudo apt-get install -y \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-force-torque-sensor-broadcaster \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro
```

EtherCAT master and development library installation depends on your system.
Confirm that CMake can find `ecrt.h` and `libethercat.so` before building this
package.

## Clone and Build

Clone the repository into the `src` directory of a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/wailers9/ati-axia-ethercat-ros2-control.git
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select ati_axia80_m20_ethercat_sensor --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## EtherCAT Preparation

Bind the IgH/EtherLab master to the host Ethernet interface connected to the ATI
sensor.

Values to confirm and configure:

| Item | Where to configure it | Notes |
| --- | --- | --- |
| EtherCAT NIC MAC address | `MASTER0_DEVICE` in `/etc/ethercat.conf` | Use the wired NIC connected to the sensor. Do not use Wi-Fi, an internet-facing NIC, or another machine's MAC address. |
| EtherCAT driver module | `DEVICE_MODULES` in `/etc/ethercat.conf` | `generic` is common. Use your EtherCAT master installation instructions if you use a dedicated driver. |
| master index | Launch argument `master_index` / xacro argument `master_index` | The first master is usually `0`. |
| slave position | Launch argument `slave_position` / xacro argument `slave_position` | Position of the ATI sensor on the EtherCAT bus. Usually `0` when it is the only slave. |
| slave alias | Launch argument `slave_alias` / xacro argument `slave_alias` | Use `0` when no EtherCAT alias is configured. |

Find the host network interfaces:

```bash
ip link
```

Find the wired interface connected to the ATI sensor, for example `enp3s0`.
Then read its MAC address:

```bash
ip link show enp3s0
```

The value after `link/ether` is the MAC address to use for `MASTER0_DEVICE`.

Edit the EtherCAT master configuration:

```bash
sudo nano /etc/ethercat.conf
```

Example configuration:

```bash
MASTER0_DEVICE="<your EtherCAT NIC MAC address>"
DEVICE_MODULES="generic"
```

Restart the EtherCAT service:

```bash
sudo systemctl restart ethercat
```

Useful checks:

```bash
systemctl status ethercat --no-pager
ethercat master
ethercat slaves
```

After the sensor is connected and the EtherCAT master is running,
`ethercat slaves` should list the ATI Axia slave. If no slave is visible, ROS 2
hardware activation will fail and no valid force data will be published.

If the bus has multiple EtherCAT slaves, confirm the sensor position:

```bash
ethercat slaves
```

The order in that list is the `slave_position`. For example, if the sensor is
the first slave, use `slave_position:=0`.

## Using the Plugin

Plugin export name:

```text
ati_axia80_m20_ethercat_sensor/Axia80M20EtherCATSensor
```

Reusable xacro:

```text
ati_axia80_m20_ethercat_sensor/urdf/ati_axia80_m20_ethercat_sensor.xacro
```

Demo URDF:

```text
ati_axia80_m20_ethercat_sensor/examples/ati_axia80_m20_demo.urdf.xacro
```

Launch the demo:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_demo.launch.py \
  master_index:=0 \
  slave_position:=0 \
  slave_alias:=0
```

Arguments:

- `master_index`: IgH/EtherLab master index, usually starting at `0`.
- `slave_position`: sensor position on the EtherCAT bus.
- `slave_alias`: EtherCAT alias. Use `0` when no alias is configured.

Relationship to `/etc/ethercat.conf`:

- `/etc/ethercat.conf` decides which host NIC the IgH/EtherLab master uses.
- `master_index` decides which IgH/EtherLab master the ROS 2 plugin requests.
- `slave_position` / `slave_alias` decide which ATI Axia slave the plugin uses
  under that master.

## Force/Torque Topic

The demo launch starts:

- `ros2_control_node`
- `robot_state_publisher`
- `force_torque_sensor_broadcaster`

Force/torque topic:

```text
/ati_axia80_m20_broadcaster/wrench
```

Inspect the topic:

```bash
ros2 topic list
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

Check hardware interfaces and controller state:

```bash
ros2 control list_hardware_interfaces
ros2 control list_controllers
```

`force_torque_sensor_broadcaster` reads these state interfaces and publishes
wrench data:

```text
ati_axia80_m20/force.x
ati_axia80_m20/force.y
ati_axia80_m20/force.z
ati_axia80_m20/torque.x
ati_axia80_m20/torque.y
ati_axia80_m20/torque.z
```

The plugin also exports:

```text
ati_axia80_m20/timestamp.sec
ati_axia80_m20/timestamp.nanosec
ati_axia80_m20/status_code
ati_axia80_m20/sample_counter
```

## Manual Bias and Clear Bias

After the sensor hardware is active, use the `std_srvs/srv/Trigger` services to
control bias manually.

Set bias:

```bash
ros2 service call /ati_axia80_m20/set_bias std_srvs/srv/Trigger '{}'
```

Clear bias:

```bash
ros2 service call /ati_axia80_m20/clear_bias std_srvs/srv/Trigger '{}'
```

Recommended workflow:

1. Start the system and wait for the sensor to settle.
2. Confirm the sensor is in the desired zero-load state.
3. Call `/<sensor_name>/set_bias`, for example `/ati_axia80_m20/set_bias`.
4. Check the output on `/ati_axia80_m20_broadcaster/wrench`.
5. To return to the unbiased output, call `/<sensor_name>/clear_bias`.

If the driver is not configured or the hardware is not active, the service
returns `success=false` with an error message.

## Main Parameters

These parameters are configured in
`urdf/ati_axia80_m20_ethercat_sensor.xacro`.

| Parameter | Default | Description |
| --- | --- | --- |
| `master_index` | `0` | IgH/EtherLab master index |
| `slave_position` | `0` | EtherCAT slave position |
| `slave_alias` | `0` | EtherCAT alias. Use `0` when no alias is configured. |
| `vendor_id` | `0x00000732` | ATI Vendor ID |
| `product_code` | `0x26483053` | Axia EtherCAT Product Code |
| `revision` | `0x00000001` | Revision |
| `read_calibration_sdo` | `true` | Read SDO calibration scales on activation |
| `runtime_diagnostic_sdo` | `true` | Read runtime SDO diagnostics on `/diagnostics` |
| `runtime_diagnostic_sdo_timeout_ms` | `5` | Runtime SDO diagnostics auto-pause threshold in milliseconds |
| `counts_per_force` | `1000000` | Manual force scale |
| `counts_per_torque` | `1000000` | Manual torque scale |
| `filter_selection` | `0` | ATI low-pass filter selection in `0x7010:01` bits 4..7. `0` disables filtering; `1..8` select progressively lower cutoff frequencies from the manual table. |
| `calibration_slot` | `0` | Calibration slot, 0..1 |
| `sample_rate_code` | `0` | 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz |
| `expected_sensor_rate_hz` | `487` | Expected sensor rate for sample-counter diagnostics |
| `expected_read_rate_hz` | `487` | Expected ROS read rate; align with `controller_manager.update_rate` |
| `clear_bias_on_activate` | `true` | Clear any existing bias on activation |
| `set_bias_on_activate` | `false` | Set bias automatically on activation. Disabled by default. |

`set_bias_on_activate` is disabled by default to avoid setting the zero point
before the sensor has stabilized.

If SDO reads fail in the field, set:

```xml
<param name="read_calibration_sdo">false</param>
```

Then manually configure `counts_per_force` and `counts_per_torque`. Both values
must be finite positive numbers and cannot be `0`.
Startup calibration SDO happens before the realtime read loop; the driver warns
if this activation-time SDO phase takes more than 500 ms.

### Low-pass Filter Selection

The Axia firmware applies filtering inside the sensor before the PDO readings
are published. The cutoff depends on both `filter_selection` and
`sample_rate_code`.

| `filter_selection` | 0.5 kHz rate | 1 kHz rate | 2 kHz rate | 4 kHz rate |
| --- | ---: | ---: | ---: | ---: |
| `0` | 200 Hz | 350 Hz | 500 Hz | 1000 Hz |
| `1` | 58 Hz | 115 Hz | 235 Hz | 460 Hz |
| `2` | 22 Hz | 45 Hz | 90 Hz | 180 Hz |
| `3` | 10 Hz | 21 Hz | 43 Hz | 84 Hz |
| `4` | 5 Hz | 10 Hz | 20 Hz | 40 Hz |
| `5` | 2.5 Hz | 5 Hz | 10 Hz | 20 Hz |
| `6` | 1.3 Hz | 3 Hz | 5 Hz | 10 Hz |
| `7` | 0.6 Hz | 1.2 Hz | 2.4 Hz | 4.7 Hz |
| `8` | 0.3 Hz | 0.7 Hz | 1.4 Hz | 2.7 Hz |

### Diagnostics and Status

ATI recommends monitoring `0x6010: Status Code` instead of the standard
EtherCAT `0x1001 Error Register`. The driver keeps the raw `uint32_t` status
word internally, exports it as the ROS 2 control `double` state interface, and
logs decoded bits only when the word changes. The decoded status bit summary is
also published in ROS diagnostics as `status_bits`.

Use these commands while the controller is active:

```bash
ros2 topic echo /diagnostics
ros2 control list_hardware_interfaces
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

Expected `/diagnostics` key values include:

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

The high-frequency `read()` cycle always performs PDO receive/process and PDO
queue/send work. On each valid PDO sample it updates `status_code`, decodes the
status bits, and checks `sample_counter` continuity by accumulating statistics
only. Diagnostics evaluates them at 1 Hz: 10/50 consecutive repeats are
WARN/ERROR; rate thresholds use twice the theoretical rate plus fixed margins
(60/100 repeats and 30/50 skipped samples per second).

EtherCAT master, domain, and slave state checks are throttled to once per second
and logged only when state changes. The first `read_once()` after activation
always performs one EtherCAT state check.

The `/diagnostics` publisher reads `0x2080: Diagnostic Readings` at 1 Hz
outside the real-time `read()` path when `runtime_diagnostic_sdo` is true. Each
runtime SDO task uses `try_lock` and does not wait for the EtherCAT driver lock.
The target runtime SDO duration is below 500 us. The driver warns if an attempt
takes more than 1 ms, and automatically pauses runtime SDO diagnostics if an
attempt exceeds `runtime_diagnostic_sdo_timeout_ms` or if five consecutive
attempts cannot acquire the driver lock. When auto-paused, `/diagnostics`
reports `runtime_sdo_auto_paused=true` and `runtime_sdo_pause_reason`.
If an SDO attempt is still running on the next diagnostics tick, the driver does
not start a second runtime SDO.
It reports external supply voltage, gage temperature, and ATI's priority status
message, including voltage faults, temperature faults, calibration checksum
errors, disconnected or out-of-range gages, force/torque range faults, hardware
or stack errors, simulated errors, and unspecified errors.

If `sample_counter` discontinuities become frequent, or `/diagnostics` reports
that runtime SDO is repeatedly skipped because the EtherCAT driver is busy,
disable runtime SDO diagnostics and rely on PDO data plus status logs:

```xml
<param name="runtime_diagnostic_sdo">false</param>
```

That same setting is the manual runtime SDO pause command. Re-enable it by
setting `runtime_diagnostic_sdo` back to `true` before launch.

## Web Monitor Dashboard

The original demo launch is still available. Use the full launch when you want
to start the sensor stack, wrench publisher, and the low-frequency monitor
dashboard together:

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py
```

After pulling this project from GitHub on a new machine, install the extra
runtime dependencies, build the package, source the workspace, then launch:

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

The monitor does not control EtherCAT and does not participate in the real-time
control path. Service, topic, and interface checks stay low-rate at 10 second
intervals by default, which is 0.1 Hz. Full dashboard telemetry such as
diagnostics, voltage, temperature, EtherCAT health, and sample-counter rates is pushed at 1 Hz by
default. Wrench data is pushed to the browser at 25 Hz by default, and the
browser redraws the six charts at 20 Hz by default. On startup, the node prints
the dashboard address once:

```text
Axia80 monitor dashboard listening on http://0.0.0.0:8765/
```

Open the dashboard from the same machine at:

```text
http://127.0.0.1:8765/
```

From another machine on the network, use the host IP address:

```text
http://<sensor-computer-ip>:8765/
```

The bind address, port, and check period can be changed at launch:

```bash
ros2 launch ati_axia80_m20_ethercat_sensor ati_axia80_m20_full.launch.py \
  monitor_host:=0.0.0.0 \
  monitor_port:=8765 \
  monitor_check_period_sec:=10.0 \
  monitor_telemetry_period_sec:=1.0 \
  monitor_wrench_push_rate_hz:=25.0 \
  monitor_chart_refresh_rate_hz:=20.0
```

Dashboard sections:

- `System Overview`: overall alert state, wrench stream health, extracted
  EtherCAT health, extracted temperature, and extracted voltage.
- `EtherCAT Health`: backend checks derived from the driver's own EtherCAT
  diagnostics and state values, including master link state, slaves responding,
  Axia slave online/operational state, status code, and runtime diagnostic SDO
  state. These values come from the same driver state checks that are logged to
  the terminal and published through `/diagnostics`.
- `Manual Bias Control`: confirmed manual calls to
  `/ati_axia80_m20/set_bias` and `/ati_axia80_m20/clear_bias`. After each call,
  the monitor continues checking that `/wrench` updates and force/torque values
  remain finite.
- `Force / Torque Channels`: six separate charts for Fx, Fy, Fz, Tx, Ty, and
  Tz, each with its latest value. WebSocket wrench updates are intended for
  20-50 Hz, and chart redraws are intended for 10-30 Hz.
- `ROS 2 Checks`: service type checks, topic type checks, wrench freshness,
  finite-value validation, expected hardware interface availability, and
  low-rate EtherCAT health summary.
- `Hardware Interfaces`: state interfaces reported by
  `/controller_manager/list_hardware_interfaces`, including availability and
  claimed state.
- `Diagnostics`: `/diagnostics` table. Diagnostic keys containing
  `temperature`, `temp`, `voltage`, or `volt` are also promoted to the overview
  metrics.

## Using It in Your Robot Description

Include this package's xacro in your robot xacro:

```xml
<xacro:include filename="$(find ati_axia80_m20_ethercat_sensor)/urdf/ati_axia80_m20_ethercat_sensor.xacro"/>

<xacro:ati_axia80_m20_ethercat_sensor
  name="ati_axia80_m20"
  parent="tool0"
  master_index="0"
  slave_position="0"
  slave_alias="0"/>
```

If your end-effector link is not `tool0`, change `parent` to the actual link
name.

## PDO Mapping

RxPDO `0x1601`:

```text
0x7010:01 Control 1
0x7010:02 Control 2
```

TxPDO `0x1A00`:

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

## Troubleshooting

- `ATI Axia EtherCAT slave was not found`: check the EtherCAT master, cable,
  power, `slave_position`, Vendor ID, and Product Code.
- No `/ati_axia80_m20_broadcaster/wrench`: check that the controller is loaded
  and active.
- Topic exists but no valid data is published: check that the hardware is active
  and that `ethercat slaves` can see the sensor.
- SDO read fails: set `read_calibration_sdo` to `false`, manually configure
  `counts_per_force` and `counts_per_torque`, then test PDO communication.
- `failed to request EtherCAT master`: check that the EtherCAT service is
  running and that the current user has permission to access the master.
