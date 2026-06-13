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
- Optionally reads force/torque scale factors from SDO `0x2021:0x37` /
  `0x2021:0x38`.
- Guards against invalid or zero `counts_per_force` and `counts_per_torque`
  values to avoid division by zero.
- Provides manual bias services:
  - `/ati_axia80_m20/set_bias`
  - `/ati_axia80_m20/clear_bias`

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
3. Call `/ati_axia80_m20/set_bias`.
4. Check the output on `/ati_axia80_m20_broadcaster/wrench`.
5. To return to the unbiased output, call `/ati_axia80_m20/clear_bias`.

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
| `counts_per_force` | `1000000` | Manual force scale |
| `counts_per_torque` | `1000000` | Manual torque scale |
| `filter_selection` | `0` | Sensor low-pass filter option, 0..8 |
| `calibration_slot` | `0` | Calibration slot, 0..1 |
| `sample_rate_code` | `0` | 0=487Hz, 1=975Hz, 2=1990Hz, 3=3900Hz |
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
