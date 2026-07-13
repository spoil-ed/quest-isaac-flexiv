import socket
import unittest

from flexiv_data_collection.protocol import JsonLineConnection, JsonLinePushClient
from flexiv_data_collection.recorder import parse_args as parse_recorder_args, resolve_task_dir


class ProtocolControlTests(unittest.TestCase):
    def test_recorder_exposes_reset_shortcut(self):
        args = parse_recorder_args(["--task-dir", "dataset"])
        self.assertEqual(args.reset_key, "r")

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
