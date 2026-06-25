#!/usr/bin/env python3

import asyncio
import json
import math
import os
import signal
import threading
import time
from collections import deque
from copy import deepcopy

from aiohttp import WSMsgType, web
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from controller_manager_msgs.srv import ListHardwareInterfaces
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


EXPECTED_STATE_INTERFACES = (
    "ati_axia80_m20/timestamp.sec",
    "ati_axia80_m20/timestamp.nanosec",
    "ati_axia80_m20/force.x",
    "ati_axia80_m20/force.y",
    "ati_axia80_m20/force.z",
    "ati_axia80_m20/torque.x",
    "ati_axia80_m20/torque.y",
    "ati_axia80_m20/torque.z",
    "ati_axia80_m20/status_code",
    "ati_axia80_m20/sample_counter",
)


class Axia80MonitorNode(Node):
    def __init__(self):
        super().__init__("axia80_monitor_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("check_period_sec", 10.0)
        self.declare_parameter("telemetry_period_sec", 1.0)
        self.declare_parameter("wrench_push_rate_hz", 25.0)
        self.declare_parameter("chart_refresh_rate_hz", 20.0)
        self.declare_parameter("wrench_topic", "/ati_axia80_m20_broadcaster/wrench")
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("set_bias_service", "/ati_axia80_m20/set_bias")
        self.declare_parameter("clear_bias_service", "/ati_axia80_m20/clear_bias")
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("wrench_stale_after_sec", 2.0)
        self.declare_parameter("history_size", 180)

        self.host = self.get_parameter("host").value
        self.port = int(self.get_parameter("port").value)
        self.check_period_sec = float(self.get_parameter("check_period_sec").value)
        self.telemetry_period_sec = float(self.get_parameter("telemetry_period_sec").value)
        self.wrench_push_rate_hz = float(self.get_parameter("wrench_push_rate_hz").value)
        self.chart_refresh_rate_hz = float(self.get_parameter("chart_refresh_rate_hz").value)
        self.wrench_topic = self.get_parameter("wrench_topic").value
        self.diagnostics_topic = self.get_parameter("diagnostics_topic").value
        self.set_bias_service_name = self.get_parameter("set_bias_service").value
        self.clear_bias_service_name = self.get_parameter("clear_bias_service").value
        controller_manager = self.get_parameter("controller_manager").value.rstrip("/")
        self.interface_service_name = f"{controller_manager}/list_hardware_interfaces"
        self.wrench_stale_after_sec = float(self.get_parameter("wrench_stale_after_sec").value)
        history_size = int(self.get_parameter("history_size").value)

        self._lock = threading.Lock()
        self._loop = None
        self._websockets = set()
        self._history = deque(maxlen=history_size)
        self._sample_counter_history = deque(maxlen=history_size)
        self._diagnostics = []
        self._diagnostic_values = {}
        self._metrics = {"temperature": None, "voltage": None}
        self._ethercat = self._derive_ethercat_health([], {})
        self._interfaces = []
        self._missing_interfaces = list(EXPECTED_STATE_INTERFACES)
        self._last_wrench_msg = None
        self._last_wrench_received_monotonic = None
        self._last_bias_action = None
        self._last_bias_result = None
        self._last_check_monotonic = None
        self._last_wrench_broadcast_monotonic = 0.0
        self._alerts = []
        self._checks = {}

        self._set_bias_client = self.create_client(Trigger, self.set_bias_service_name)
        self._clear_bias_client = self.create_client(Trigger, self.clear_bias_service_name)
        self._interfaces_client = self.create_client(
            ListHardwareInterfaces, self.interface_service_name
        )

        self.create_subscription(WrenchStamped, self.wrench_topic, self._on_wrench, 10)
        self.create_subscription(DiagnosticArray, self.diagnostics_topic, self._on_diagnostics, 10)

        period = max(0.1, self.check_period_sec)
        self.create_timer(period, self._run_low_frequency_checks)
        telemetry_period = max(0.1, self.telemetry_period_sec)
        self.create_timer(telemetry_period, self._schedule_full_broadcast)

        self._run_low_frequency_checks()
        self.get_logger().info(
            f"Axia80 monitor dashboard listening on http://{self.host}:{self.port}/"
        )

    def set_event_loop(self, loop):
        self._loop = loop

    def add_websocket(self, ws):
        self._websockets.add(ws)

    def remove_websocket(self, ws):
        self._websockets.discard(ws)

    def dashboard_state(self):
        with self._lock:
            return {
                "config": {
                    "check_period_sec": self.check_period_sec,
                    "telemetry_period_sec": self.telemetry_period_sec,
                    "wrench_push_rate_hz": self.wrench_push_rate_hz,
                    "chart_refresh_rate_hz": self.chart_refresh_rate_hz,
                    "wrench_topic": self.wrench_topic,
                    "diagnostics_topic": self.diagnostics_topic,
                    "set_bias_service": self.set_bias_service_name,
                    "clear_bias_service": self.clear_bias_service_name,
                    "interface_service": self.interface_service_name,
                    "wrench_stale_after_sec": self.wrench_stale_after_sec,
                },
                "checks": deepcopy(self._checks),
                "alerts": list(self._alerts),
                "wrench": self._wrench_snapshot_locked(),
                "history": list(self._history),
                "sample_counter_history": list(self._sample_counter_history),
                "diagnostics": deepcopy(self._diagnostics),
                "diagnostic_values": deepcopy(self._diagnostic_values),
                "metrics": deepcopy(self._metrics),
                "ethercat": deepcopy(self._ethercat),
                "interfaces": deepcopy(self._interfaces),
                "missing_interfaces": list(self._missing_interfaces),
                "last_bias_action": deepcopy(self._last_bias_action),
                "last_bias_result": deepcopy(self._last_bias_result),
                "server_time": time.time(),
            }

    async def trigger_bias(self, action):
        if action == "set":
            client = self._set_bias_client
            service_name = self.set_bias_service_name
        elif action == "clear":
            client = self._clear_bias_client
            service_name = self.clear_bias_service_name
        else:
            return {"success": False, "message": f"Unsupported bias action: {action}"}

        started = time.time()
        with self._lock:
            self._last_bias_action = {"action": action, "time": started, "service": service_name}

        if not client.wait_for_service(timeout_sec=0.0):
            result = {"success": False, "message": f"Service not available: {service_name}"}
            self._store_bias_result(action, result)
            self._run_low_frequency_checks()
            return result

        ros_future = client.call_async(Trigger.Request())
        loop = asyncio.get_running_loop()
        aio_future = loop.create_future()

        def complete(future):
            try:
                response = future.result()
                payload = {"success": bool(response.success), "message": response.message}
            except Exception as exc:
                payload = {"success": False, "message": str(exc)}
            loop.call_soon_threadsafe(aio_future.set_result, payload)

        ros_future.add_done_callback(complete)

        try:
            result = await asyncio.wait_for(aio_future, timeout=5.0)
        except asyncio.TimeoutError:
            result = {"success": False, "message": f"Timed out calling {service_name}"}

        self._store_bias_result(action, result)
        self._run_low_frequency_checks()
        self._schedule_full_broadcast()
        return result

    def _store_bias_result(self, action, result):
        with self._lock:
            self._last_bias_result = {
                "action": action,
                "time": time.time(),
                "success": bool(result.get("success")),
                "message": result.get("message", ""),
            }

    def _on_wrench(self, msg):
        values = self._wrench_values(msg)
        received = time.monotonic()
        sample = {
            "time": time.time(),
            "fx": values[0],
            "fy": values[1],
            "fz": values[2],
            "tx": values[3],
            "ty": values[4],
            "tz": values[5],
            "valid": all(math.isfinite(v) for v in values),
            "frame_id": msg.header.frame_id,
        }
        with self._lock:
            self._last_wrench_msg = msg
            self._last_wrench_received_monotonic = received
            self._history.append(sample)
        self._schedule_wrench_broadcast()

    def _on_diagnostics(self, msg):
        rows = []
        flat_values = {}
        metrics = {"temperature": None, "voltage": None}
        for status in msg.status:
            values = {}
            for item in status.values:
                values[item.key] = item.value
                flat_values.setdefault(item.key, item.value)
                key = item.key.lower()
                if "temperature" in key or key == "temp" or key.endswith(".temp"):
                    metrics["temperature"] = item.value
                if "voltage" in key or "volt" in key:
                    metrics["voltage"] = item.value
            rows.append(
                {
                    "name": status.name,
                    "hardware_id": status.hardware_id,
                    "level": self._diagnostic_level(status.level),
                    "message": status.message,
                    "values": values,
                }
            )

        ethercat = self._derive_ethercat_health(rows, flat_values)
        counter_sample = self._sample_counter_sample(flat_values)
        with self._lock:
            self._diagnostics = rows
            self._diagnostic_values = flat_values
            self._ethercat = ethercat
            if metrics["temperature"] is not None:
                self._metrics["temperature"] = metrics["temperature"]
            if metrics["voltage"] is not None:
                self._metrics["voltage"] = metrics["voltage"]
            if counter_sample is not None:
                self._sample_counter_history.append(counter_sample)
        self._schedule_full_broadcast()

    def _sample_counter_sample(self, values):
        repeats = self._parse_number(values.get("actual_repeats_per_sec"))
        jumps = self._parse_number(values.get("actual_jump_events_per_sec"))
        skipped = self._parse_number(values.get("actual_skipped_samples_per_sec"))
        if repeats is None and jumps is None and skipped is None:
            return None
        return {
            "time": time.time(),
            "actual_repeats_per_sec": repeats,
            "actual_jump_events_per_sec": jumps,
            "actual_skipped_samples_per_sec": skipped,
            "expected_repeats_per_sec": self._parse_number(
                values.get("expected_repeats_per_sec")
            ),
            "expected_skipped_samples_per_sec": self._parse_number(
                values.get("expected_skipped_samples_per_sec")
            ),
            "expected_jump_events_per_sec": self._parse_number(
                values.get("expected_jump_events_per_sec")
            ),
        }

    def _run_low_frequency_checks(self):
        topics = dict(self.get_topic_names_and_types())
        services = dict(self.get_service_names_and_types())
        set_bias_ok = self._has_type(services, self.set_bias_service_name, "std_srvs/srv/Trigger")
        clear_bias_ok = self._has_type(
            services, self.clear_bias_service_name, "std_srvs/srv/Trigger"
        )
        wrench_topic_ok = self._has_type(
            topics, self.wrench_topic, "geometry_msgs/msg/WrenchStamped"
        )
        diagnostics_topic_ok = self._has_type(
            topics, self.diagnostics_topic, "diagnostic_msgs/msg/DiagnosticArray"
        )
        wrench_snapshot = self._wrench_snapshot()
        ethercat = self._ethercat_snapshot()
        with self._lock:
            diagnostics = deepcopy(self._diagnostics)

        checks = {
            "set_bias_service": self._check_row(
                set_bias_ok, self._type_detail(services, self.set_bias_service_name)
            ),
            "clear_bias_service": self._check_row(
                clear_bias_ok, self._type_detail(services, self.clear_bias_service_name)
            ),
            "wrench_topic": self._check_row(
                wrench_topic_ok, self._type_detail(topics, self.wrench_topic)
            ),
            "diagnostics_topic": self._check_row(
                diagnostics_topic_ok, self._type_detail(topics, self.diagnostics_topic)
            ),
            "wrench_updating": self._check_row(
                wrench_snapshot["updating"], f"last age {wrench_snapshot['age_sec']} sec"
            ),
            "wrench_values_valid": self._check_row(
                wrench_snapshot["valid"], "force/torque finite numeric values"
            ),
            "hardware_interfaces": self._check_row(
                len(self._missing_interfaces) == 0,
                f"missing {len(self._missing_interfaces)} expected state interfaces",
            ),
            "ethercat_health": self._check_row(ethercat["ok"], ethercat["summary"]),
        }
        diagnostic_names = {
            "Axia80 EtherCAT communication": "diagnostic_ethercat_communication",
            "Axia80 sample counter": "diagnostic_sample_counter",
            "Axia80 runtime SDO diagnostics": "diagnostic_runtime_sdo",
            "Axia80 sensor status code": "diagnostic_sensor_status",
        }
        for label, check_name in diagnostic_names.items():
            row = next((item for item in diagnostics if item["name"].endswith(label)), None)
            checks[check_name] = self._check_row(
                row is not None and row["level"] == 0,
                row["message"] if row is not None else f"{label} not reported",
            )

        alerts = []
        for name, check in checks.items():
            if not check["ok"]:
                alerts.append(f"{name}: {check['detail']}")

        with self._lock:
            self._checks = checks
            self._alerts = alerts
            self._last_check_monotonic = time.monotonic()

        if self._interfaces_client.wait_for_service(timeout_sec=0.0):
            future = self._interfaces_client.call_async(ListHardwareInterfaces.Request())
            future.add_done_callback(self._on_interfaces)

        self._log_alerts(alerts)
        self._schedule_full_broadcast()

    def _on_interfaces(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to query hardware interfaces: {exc}")
            return

        rows = []
        names = set()
        for interface in response.state_interfaces:
            rows.append(
                {
                    "name": interface.name,
                    "data_type": interface.data_type,
                    "available": bool(interface.is_available),
                    "claimed": bool(interface.is_claimed),
                }
            )
            if interface.is_available:
                names.add(interface.name)

        missing = [name for name in EXPECTED_STATE_INTERFACES if name not in names]
        with self._lock:
            self._interfaces = rows
            self._missing_interfaces = missing
            if "hardware_interfaces" in self._checks:
                self._checks["hardware_interfaces"] = self._check_row(
                    len(missing) == 0, f"missing {len(missing)} expected state interfaces"
                )
            self._alerts = [
                alert for alert in self._alerts if not alert.startswith("hardware_interfaces:")
            ]
            if missing:
                self._alerts.append(
                    f"hardware_interfaces: missing {len(missing)} expected state interfaces"
                )
        self._schedule_full_broadcast()

    def _wrench_snapshot(self):
        with self._lock:
            return self._wrench_snapshot_locked()

    def _ethercat_snapshot(self):
        with self._lock:
            return deepcopy(self._ethercat)

    def _wrench_snapshot_locked(self):
        if self._last_wrench_msg is None:
            return {
                "received": False,
                "updating": False,
                "valid": False,
                "age_sec": None,
                "values": {},
                "frame_id": "",
            }

        age_sec = time.monotonic() - self._last_wrench_received_monotonic
        values = self._wrench_values(self._last_wrench_msg)
        return {
            "received": True,
            "updating": age_sec <= self.wrench_stale_after_sec,
            "valid": all(math.isfinite(v) for v in values),
            "age_sec": round(age_sec, 3),
            "values": {
                "fx": values[0],
                "fy": values[1],
                "fz": values[2],
                "tx": values[3],
                "ty": values[4],
                "tz": values[5],
            },
            "frame_id": self._last_wrench_msg.header.frame_id,
        }

    @staticmethod
    def _wrench_values(msg):
        return (
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
        )

    @staticmethod
    def _check_row(ok, detail):
        return {"ok": bool(ok), "detail": detail, "time": time.time()}

    @staticmethod
    def _has_type(graph_items, name, expected_type):
        return expected_type in graph_items.get(name, [])

    @staticmethod
    def _type_detail(graph_items, name):
        types = graph_items.get(name, [])
        if not types:
            return f"{name} not found"
        return f"{name}: {', '.join(types)}"

    def _derive_ethercat_health(self, rows, values):
        items = []

        def add_item(name, ok, detail):
            items.append({"name": name, "ok": bool(ok), "detail": detail})

        diagnostic_seen = bool(rows)
        add_item(
            "driver_diagnostics",
            diagnostic_seen,
            "driver diagnostics received" if diagnostic_seen else "no /diagnostics message yet",
        )

        link_up = self._parse_bool(values.get("ethercat_master_link_up"))
        add_item(
            "master_link",
            link_up is True,
            self._value_detail("ethercat_master_link_up", values.get("ethercat_master_link_up")),
        )

        slaves = values.get("ethercat_master_slaves_responding")
        add_item(
            "slaves_responding",
            self._parse_number(slaves) is not None and self._parse_number(slaves) > 0,
            self._value_detail("ethercat_master_slaves_responding", slaves),
        )

        slave_online = self._parse_bool(values.get("ethercat_slave_online"))
        add_item(
            "slave_online",
            slave_online is True,
            self._value_detail("ethercat_slave_online", values.get("ethercat_slave_online")),
        )

        slave_operational = self._parse_bool(values.get("ethercat_slave_operational"))
        add_item(
            "slave_operational",
            slave_operational is True,
            self._value_detail(
                "ethercat_slave_operational", values.get("ethercat_slave_operational")
            ),
        )

        ok = all(item["ok"] for item in items)
        bad_items = [item["name"] for item in items if not item["ok"]]
        summary = "EtherCAT diagnostics OK" if ok else "EtherCAT issue: " + ", ".join(bad_items)
        return {
            "ok": ok,
            "summary": summary,
            "items": items,
            "values": {
                key: values.get(key)
                for key in (
                    "ethercat_domain_working_counter",
                    "ethercat_domain_wc_state",
                    "ethercat_master_slaves_responding",
                    "ethercat_master_al_states",
                    "ethercat_master_link_up",
                    "ethercat_slave_online",
                    "ethercat_slave_operational",
                    "ethercat_slave_al_state",
                )
            },
        }

    @staticmethod
    def _value_detail(name, value):
        if value is None or value == "":
            return f"{name} not reported"
        return f"{name}={value}"

    @staticmethod
    def _diagnostic_level(value):
        if isinstance(value, (bytes, bytearray)):
            return value[0] if value else 0
        return int(value)

    @staticmethod
    def _parse_bool(value):
        if value is None:
            return None
        text = str(value).strip().lower()
        if text in {"true", "1", "yes", "up", "online", "operational"}:
            return True
        if text in {"false", "0", "no", "down", "offline"}:
            return False
        return None

    @staticmethod
    def _parse_number(value):
        try:
            return float(str(value).strip())
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _parse_int(value):
        try:
            return int(str(value).strip(), 0)
        except (TypeError, ValueError):
            return None

    def _log_alerts(self, alerts):
        if alerts:
            self.get_logger().warning("Axia80 monitor alerts: " + "; ".join(alerts))
        else:
            self.get_logger().info("Axia80 monitor checks OK")

    def _schedule_full_broadcast(self):
        if self._loop is None or not self._websockets:
            return
        payload = {"type": "full", "state": self.dashboard_state()}
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    def _schedule_wrench_broadcast(self):
        if self._loop is None or not self._websockets:
            return
        now = time.monotonic()
        min_period = 1.0 / max(1.0, self.wrench_push_rate_hz)
        if now - self._last_wrench_broadcast_monotonic < min_period:
            return
        self._last_wrench_broadcast_monotonic = now
        with self._lock:
            payload = {
                "type": "wrench",
                "wrench": self._wrench_snapshot_locked(),
                "history": list(self._history),
                "server_time": time.time(),
            }
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, message):
        payload = json.dumps(message)
        stale = []
        for ws in list(self._websockets):
            try:
                await ws.send_str(payload)
            except Exception:
                stale.append(ws)
        for ws in stale:
            self.remove_websocket(ws)


def dashboard_dir():
    try:
        return os.path.join(
            get_package_share_directory("ati_axia80_m20_ethercat_sensor"), "dashboard"
        )
    except PackageNotFoundError:
        return os.path.join(os.path.dirname(os.path.dirname(__file__)), "dashboard")


def make_app(node):
    app = web.Application()
    static_dir = dashboard_dir()

    async def index(_request):
        return web.FileResponse(os.path.join(static_dir, "index.html"))

    async def static_file(request):
        filename = request.match_info["filename"]
        if filename not in {"dashboard.js", "styles.css"}:
            raise web.HTTPNotFound()
        return web.FileResponse(os.path.join(static_dir, filename))

    async def state(_request):
        return web.json_response(node.dashboard_state())

    async def bias(request):
        action = request.match_info["action"]
        result = await node.trigger_bias(action)
        return web.json_response(result, status=200 if result["success"] else 503)

    async def websocket(request):
        ws = web.WebSocketResponse(heartbeat=30)
        await ws.prepare(request)
        node.add_websocket(ws)
        await ws.send_str(json.dumps({"type": "full", "state": node.dashboard_state()}))
        async for message in ws:
            if message.type == WSMsgType.ERROR:
                break
        node.remove_websocket(ws)
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/api/state", state)
    app.router.add_post("/api/bias/{action:set|clear}", bias)
    app.router.add_get("/ws", websocket)
    app.router.add_get("/static/{filename}", static_file)
    return app


async def run_web(node):
    app = make_app(node)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, node.host, node.port)
    await site.start()

    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, stop.set)
        except NotImplementedError:
            pass
    await stop.wait()
    await runner.cleanup()


def main():
    rclpy.init()
    node = Axia80MonitorNode()
    loop = asyncio.new_event_loop()
    node.set_event_loop(loop)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        loop.run_until_complete(run_web(node))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        loop.close()


if __name__ == "__main__":
    main()
