# Copyright 2026 Mowgli Project
# SPDX-License-Identifier: GPL-3.0

import importlib.util
from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def _load_module(filename: str, module_name: str):
    here = Path(__file__).resolve().parent
    path = here.parent / "launch" / filename
    spec = importlib.util.spec_from_file_location(module_name, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_full_system_show_args_no_longer_exposes_internal_universal_toggle() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch_args")
    launch_description = launch_module.generate_launch_description()

    declared_args = [
        entity.name
        for entity in launch_description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]

    assert "use_universal_gnss" not in declared_args


def test_full_system_no_longer_includes_internal_universal_launch() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch_includes")
    launch_description = launch_module.generate_launch_description()

    included_locations = [
        entity.launch_description_source.location
        for entity in launch_description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert all(not location.endswith("universal_gnss.launch.py") for location in included_locations)


def test_full_system_disables_local_status_when_universal_selected() -> None:
    launch_module = _load_module("full_system.launch.py", "full_system_launch")
    assert launch_module._local_gnss_status_enabled("mowgli_local") is False
    assert launch_module._local_gnss_status_enabled("universal") is False
    assert launch_module._local_gnss_status_enabled("external") is False
    assert launch_module._local_gnss_status_enabled("off") is False


def test_sim_full_system_matches_runtime_status_switch() -> None:
    launch_module = _load_module("sim_full_system.launch.py", "sim_full_system_launch")
    assert launch_module._local_gnss_status_enabled("gps") is False
    assert launch_module._local_gnss_status_enabled("universal") is False
