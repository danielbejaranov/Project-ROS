#!/usr/bin/env python3
"""Tanda estática: reutiliza el grabador ROS nativo con dos vistas y cinco etiquetas."""
import importlib.util
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
spec = importlib.util.spec_from_file_location('native_recording', ROOT/'ros2_validation/run_real_capture.py')
recording = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = recording
spec.loader.exec_module(recording)
original_launch_source = recording.launch_source
original_arguments = recording.arguments


def arguments():
    return original_arguments({
        'controller_profile': 'obstacles',
        'map_stem': ROOT/'results/Sakhir_obstacles_5',
        'gain': .7, 'lateral_accel': 6., 'brake_accel': 6.,
        'smooth': 11, 'alpha': .8, 'display': ':189', 'domain': 189,
        'timeout_wall': 1800.,
    })


def launch_source(config, share, map_yaml):
    text = original_launch_source(config, share, map_yaml)
    text = text.replace('from launch import LaunchDescription',
                        'from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess')
    for name in ('overview', 'follow'):
        text = text.replace(repr(str(ROOT/f'ros2_validation/{name}.rviz')),
                            repr(str(HERE/f'static_{name}.rviz')))
    marker = f"        ExecuteProcess(cmd=[{sys.executable!r}, {str(HERE/'obstacle_markers.py')!r}, {str(HERE/'static_obstacles.yaml')!r}], output='screen'),\n"
    text = text.replace('    ])', marker + '    ])')
    return text


recording.launch_source = launch_source
recording.arguments = arguments

if __name__ == '__main__':
    raise SystemExit(recording.main())
