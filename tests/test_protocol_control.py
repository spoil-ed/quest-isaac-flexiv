import socket
import json
import unittest
from unittest import mock

from flexiv_data_collection.gateway import LatestBridgeData
from flexiv_data_collection import recorder
from flexiv_data_collection.protocol import JsonLineConnection, JsonLinePushClient
from flexiv_data_collection.recorder import (
    FlexivEpisodeWriter,
    format_duration,
    parse_args as parse_recorder_args,
    reset_status_from_sample,
    resolve_task_dir,
)


class ProtocolControlTests(unittest.TestCase):
    def test_recorder_formats_duration(self):
        self.assertEqual(format_duration(0), "00:00:00")
        self.assertEqual(format_duration(3661.4), "01:01:01")

    def test_recorder_summarizes_saved_task_episodes(self):
        from tempfile import TemporaryDirectory
        from pathlib import Path

        with TemporaryDirectory() as tmp:
            task_dir = Path(tmp) / "task"
            episode_dir = task_dir / "episode_001"
            episode_dir.mkdir(parents=True)
            payload = {
                "info": {"image": {"fps": 10}},
                "data": [{"idx": 0}, {"idx": 1}, {"idx": 2}],
            }
            (episode_dir / "data.json").write_text(
                json.dumps(payload, indent=4), encoding="utf-8"
            )
            writer = FlexivEpisodeWriter(
                task_dir,
                fps=30,
                image_size=(640, 480),
                task_goal="goal",
                task_desc="desc",
                task_steps="steps",
            )

            episodes, frames, duration = writer.saved_task_summary()

        self.assertEqual((episodes, frames), (1, 3))
        self.assertAlmostEqual(duration, 0.3)

    def test_recorder_exposes_reset_shortcut(self):
        args = parse_recorder_args(["--task-dir", "dataset"])
        self.assertEqual(args.start_key, "s")
        self.assertEqual(args.stop_key, "e")
        self.assertEqual(args.reset_key, "r")
        self.assertEqual(args.reset_key_cooldown_sec, 2.5)
        self.assertEqual(args.reset_timeout_sec, 25.0)

    def test_recorder_extracts_bridge_reset_status(self):
        status = reset_status_from_sample(
            {
                "sim_state": {
                    "bridge": {
                        "reset": {"last_seq": 4, "state": "succeeded", "ready": True}
                    }
                }
            }
        )

        self.assertEqual(status["last_seq"], 4)
        self.assertTrue(status["ready"])

    def test_recorder_waits_for_matching_reset_success(self):
        samples = [
            {"sim_state": {"bridge": {"reset": {"last_seq": 1, "state": "moving"}}}},
            {
                "sim_state": {
                    "bridge": {
                        "reset": {"last_seq": 2, "state": "succeeded", "ready": True}
                    }
                }
            },
        ]
        with mock.patch.object(recorder, "request_sample", side_effect=samples):
            status = recorder.wait_for_reset(object(), expected_seq=2, timeout_sec=1.0)

        self.assertEqual(status["state"], "succeeded")

    def test_recorder_preflight_returns_historical_reset_failure_for_recovery(self):
        failed = {
            "sim_state": {
                "bridge": {
                    "reset": {
                        "last_seq": 2,
                        "state": "failed",
                        "ready": False,
                        "error": "old timeout",
                    }
                }
            }
        }
        with mock.patch.object(recorder, "request_sample", return_value=failed):
            status = recorder.wait_for_reset(object(), expected_seq=None, timeout_sec=1.0)

        self.assertEqual(status["state"], "failed")

    def test_recorder_matching_reset_failure_still_raises(self):
        failed = {
            "sim_state": {
                "bridge": {
                    "reset": {
                        "last_seq": 3,
                        "state": "failed",
                        "ready": False,
                        "error": "current timeout",
                    }
                }
            }
        }
        with mock.patch.object(recorder, "request_sample", return_value=failed):
            with self.assertRaisesRegex(RuntimeError, "current timeout"):
                recorder.wait_for_reset(object(), expected_seq=3, timeout_sec=1.0)

    def test_gateway_rejects_overlapping_reset(self):
        latest = LatestBridgeData()
        latest.update({"sim_state": {"reset": {"state": "moving"}}})

        with self.assertRaisesRegex(RuntimeError, "already in progress"):
            latest.request_reset("keyboard")

    def test_gateway_keeps_reset_inflight_until_terminal_status(self):
        latest = LatestBridgeData()
        first = latest.request_reset("keyboard")
        latest.consume_reset_request()

        with self.assertRaisesRegex(RuntimeError, "already in progress"):
            latest.request_reset("save")

        latest.update(
            {
                "sim_state": {
                    "reset": {"last_seq": first["seq"], "state": "succeeded", "ready": True}
                }
            }
        )
        second = latest.request_reset("save")
        self.assertEqual(second["seq"], first["seq"] + 1)

    def test_recorder_task_name_resolves_under_output_root(self):
        args = parse_recorder_args(
            ["--task-name", "pick_cube", "--output-root", "datasets/stage1_records"]
        )
        self.assertEqual(resolve_task_dir(args).as_posix(), "datasets/stage1_records/pick_cube")

    def test_recorder_rejects_task_name_with_path_separator(self):
        with self.assertRaises(SystemExit):
            parse_recorder_args(["--task-name", "group/pick_cube"])

    def test_push_client_receives_optional_control_without_blocking(self):
        client_socket, server_socket = socket.socketpair()
        client = JsonLinePushClient.__new__(JsonLinePushClient)
        client.endpoint = "socketpair"
        client.timeout = 0.0
        client.retry = False
        client.connection = JsonLineConnection(client_socket)
        server = JsonLineConnection(server_socket)
        try:
            self.assertIsNone(client.recv_json_if_available())
            server.send_json({"type": "flexiv_bridge_control", "command": "reset", "seq": 1})
            self.assertEqual(client.recv_json_if_available(timeout=0.1)["command"], "reset")
        finally:
            client.close()
            server.close()


if __name__ == "__main__":
    unittest.main()
