# ATI Axia EtherCAT ROS 2 Control

ROS 2 Jazzy `ros2_control` sensor plugin for the ATI Axia80-M20 force/torque
sensor over IgH/EtherLab EtherCAT.

The package reads force/torque data through EtherCAT PDOs and publishes wrench
data through `force_torque_sensor_broadcaster`.

Quick topic check after launch:

```bash
ros2 topic echo /ati_axia80_m20_broadcaster/wrench
```

Manual bias services:

```bash
ros2 service call /ati_axia80_m20/set_bias std_srvs/srv/Trigger '{}'
ros2 service call /ati_axia80_m20/clear_bias std_srvs/srv/Trigger '{}'
```

See [ati_axia80_m20_ethercat_sensor/README.md](ati_axia80_m20_ethercat_sensor/README.md)
for clone, build, launch, topic, and troubleshooting instructions.
