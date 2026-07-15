from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "start_robot_control_app.py"


def load_module():
    spec = importlib.util.spec_from_file_location("start_robot_control_app_test", MODULE_PATH)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_discovery_infers_encrypted_specs_path_on_cold_start(tmp_path: Path) -> None:
    module = load_module()
    param = (
        tmp_path
        / "user_data_ui"
        / "simDir"
        / "simulator0"
        / "A02L-00-M6-I0LIRN"
        / "arm_driver_param.xml"
    )
    param.parent.mkdir(parents=True)
    param.write_text("<params/>", encoding="utf-8")

    discovered = module.discover_robot_control_args(tmp_path)

    assert discovered == {
        "serial": "A02L-00-M6-I0LIRN",
        "config": "specs/robots/FlexivA02L/flexivCfg.xml",
    }


def test_discovery_prefers_existing_specs_config(tmp_path: Path) -> None:
    module = load_module()
    param = (
        tmp_path
        / "user_data_ui"
        / "simDir"
        / "simulator0"
        / "A02L-00-M6-I0LIRN"
        / "arm_driver_param.xml"
    )
    param.parent.mkdir(parents=True)
    param.write_text("<params/>", encoding="utf-8")
    config = tmp_path / "specs" / "robots" / "CustomA02L" / "flexivCfg.xml"
    config.parent.mkdir(parents=True)
    config.write_text("<config/>", encoding="utf-8")

    discovered = module.discover_robot_control_args(tmp_path)

    assert discovered["config"] == "specs/robots/CustomA02L/flexivCfg.xml"


def test_capsh_launcher_passes_sys_nice_as_an_ambient_capability(tmp_path: Path) -> None:
    module = load_module()
    args = module.parse_args(
        [
            "--studio-root",
            str(tmp_path),
            "--serial",
            "A02L-00-M6-I0LIRN",
            "--config",
            "specs/robots/FlexivA02L/flexivCfg.xml",
            "--capsh",
            str(tmp_path / "runtime-tools" / "capsh"),
        ]
    )

    command = module.build_command(args)

    assert command[0] == str((tmp_path / "runtime-tools" / "capsh").resolve())
    assert "--caps=cap_sys_nice+eip" in command
    assert "--addamb=cap_sys_nice" in command
    assert 'export LD_LIBRARY_PATH="$PWD/lib' in command[5]
    assert command[-13:] == [
        "./RobotControlApp",
        "-u",
        "./user_data_ui//./simDir/simulator0/user_data/",
        "-c",
        "specs/robots/FlexivA02L/flexivCfg.xml",
        "-m",
        "MotionBarSimulation",
        "-s",
        "A02L-00-M6-I0LIRN",
        "-x",
        "CX01-02-P1-00034",
        "-n",
        "-g",
    ]
