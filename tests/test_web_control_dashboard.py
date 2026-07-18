import importlib.util
import json
import socket
import threading
import unittest
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/web_control_dashboard.py"
SPEC = importlib.util.spec_from_file_location("web_control_dashboard", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class WebControlDashboardTests(unittest.TestCase):
    def test_dashboard_status_marks_fresh_packets_online(self):
        dashboard = MODULE.DashboardState("127.0.0.1", 57687, timeout_sec=2.0)
        dashboard.arm.update({"schema": MODULE.ARM_STATE_SCHEMA, "arms": {}})

        status = dashboard.status()

        self.assertTrue(status["arm"]["online"])
        self.assertFalse(status["recorder"]["online"])

    def test_dashboard_sends_validated_recorder_command(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1.0)
        dashboard = MODULE.DashboardState(
            "127.0.0.1", receiver.getsockname()[1], timeout_sec=2.0
        )
        try:
            dashboard.send_recorder_command("start")
            packet = json.loads(receiver.recvfrom(4096)[0])
        finally:
            receiver.close()

        self.assertEqual(packet, {"command": "start"})
        with self.assertRaisesRegex(ValueError, "unsupported"):
            dashboard.send_recorder_command("shell")

    def test_dashboard_html_contains_health_and_record_controls(self):
        self.assertIn("Flexiv 双臂采集台", MODULE.DASHBOARD_HTML)
        self.assertIn("command('start')", MODULE.DASHBOARD_HTML)
        self.assertIn("command('save')", MODULE.DASHBOARD_HTML)
        self.assertIn("RESET 双臂 + 环境", MODULE.DASHBOARD_HTML)
        self.assertIn('action="/api/reset"', MODULE.DASHBOARD_HTML)
        self.assertIn('type="submit"', MODULE.DASHBOARD_HTML)
        self.assertNotIn("command('reset',true)", MODULE.DASHBOARD_HTML)
        self.assertIn("questSpacing", MODULE.DASHBOARD_HTML)
        self.assertNotIn("questDirection", MODULE.DASHBOARD_HTML)
        self.assertNotIn("相对位姿", MODULE.DASHBOARD_HTML)
        self.assertIn("calibration_geometry", MODULE.DASHBOARD_HTML)
        self.assertIn("保持 squeeze 直接跟随", MODULE.DASHBOARD_HTML)
        self.assertIn("leftTable", MODULE.DASHBOARD_HTML)
        self.assertIn("实时最大关节力矩风险", MODULE.DASHBOARD_HTML)
        self.assertIn("if(refreshing)return", MODULE.DASHBOARD_HTML)
        self.assertIn("font-variant-numeric:tabular-nums", MODULE.DASHBOARD_HTML)
        self.assertNotIn("<td>${num(q[i],4)}</td>", MODULE.DASHBOARD_HTML)

    def test_dashboard_default_ports_match_collection_stack(self):
        args = MODULE.parse_args([])

        self.assertEqual(args.port, 8080)
        self.assertEqual(args.arm_state_port, 57684)
        self.assertEqual(args.recorder_command_port, 57687)
        self.assertEqual(args.recorder_status_port, 57688)

    def test_native_reset_form_endpoint_sends_reset_without_javascript(self):
        receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1.0)
        dashboard = MODULE.DashboardState(
            "127.0.0.1", receiver.getsockname()[1], timeout_sec=2.0
        )
        server = MODULE.DashboardHttpServer(("127.0.0.1", 0), dashboard)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            url = f"http://127.0.0.1:{server.server_address[1]}/api/reset"
            with urllib.request.urlopen(urllib.request.Request(url, method="POST")) as response:
                self.assertEqual(response.status, 202)
            packet = json.loads(receiver.recvfrom(4096)[0])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=1.0)
            receiver.close()

        self.assertEqual(packet, {"command": "reset"})


if __name__ == "__main__":
    unittest.main()
