#!/usr/bin/env python3
"""Run an isolated ten-lap candidate with native ROS 2, RViz and DDS evidence.

Source /opt/ros/humble/setup.bash and the native workspace overlay first.
Every run requires a new --out directory; previous results are never replaced.
The source simulator configuration and vehicle physics are left untouched.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time


def positive(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError("must be finite and greater than zero")
    return number


def fraction(value: str) -> float:
    number = float(value)
    if not math.isfinite(number) or not 0 <= number <= 1:
        raise argparse.ArgumentTypeError("must be in [0, 1]")
    return number


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--mode", choices=("lockstep", "wall"), default="lockstep")
    parser.add_argument("--max-speed", required=True, type=positive)
    parser.add_argument("--controller-profile", default="fast")
    parser.add_argument("--gain", type=positive, default=.7)
    parser.add_argument("--lateral-accel", type=positive, default=12.)
    parser.add_argument("--brake-accel", type=positive, default=8.)
    parser.add_argument("--smooth", type=int, default=31)
    parser.add_argument("--alpha", type=fraction, default=.6)
    parser.add_argument("--display", default=":109")
    parser.add_argument("--domain", type=int, default=93)
    parser.add_argument("--record-video", action="store_true")
    parser.add_argument('--start-file', type=Path, required=True, help='Controller begins only after preflight is reviewed and this file is created.')
    parser.add_argument("--map-stem", type=Path,
                        default=Path(os.environ.get('DANIEL_F1TENTH_MAP',
                                     str(Path(__file__).resolve().parents[1] / 'results/Sakhir_map'))),
                        help="Mapa sin extensión; por defecto results/Sakhir_map del clon")
    parser.add_argument("--timeout-wall", type=positive,
                        help="default: 600 seconds in lockstep; 1200 seconds in wall mode")
    parser.add_argument("--startup-seconds", type=positive, default=8.)
    args = parser.parse_args()
    if not re.fullmatch(r":\d+", args.display):
        parser.error("--display must name a private local display, e.g. :109")
    if not 0 <= args.domain <= 232:
        parser.error("--domain must be between 0 and 232")
    if args.smooth < 1:
        parser.error("--smooth must be an integer greater than zero")
    if args.timeout_wall is None:
        args.timeout_wall = 600. if args.mode == 'lockstep' else 1200.
    return args


def digest(path: Path) -> str:
    sha = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(chunk)
    return sha.hexdigest()


def launch_source(config: Path, share: Path, map_yaml: Path) -> str:
    """A per-run infrastructure launch; the controller starts after recording."""
    visual_dir = Path(__file__).resolve().parent
    return f'''"""Generated infrastructure for this candidate only."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz_overview',
             parameters=[{{'use_sim_time': True}}],
             arguments=['-d', {str(visual_dir / 'overview.rviz')!r}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz_detalle',
             parameters=[{{'use_sim_time': True}}],
             arguments=['-d', {str(visual_dir / 'follow.rviz')!r}], output='screen'),
        Node(package='f1tenth_gym_ros', executable='gym_bridge', name='bridge',
             parameters=[{str(config)!r}], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{{'yaml_filename': {str(map_yaml)!r}, 'topic': 'map',
                          'frame_id': 'map', 'use_sim_time': True}}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[{{'use_sim_time': True, 'autostart': True,
                          'node_names': ['map_server']}}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='ego_robot_state_publisher',
             parameters=[{{'use_sim_time': True, 'robot_description': Command(
                 ['xacro ', {str(share / 'launch/ego_racecar.xacro')!r}])}}],
             remappings=[('/robot_description', 'ego_robot_description')], output='screen'),
    ])
'''


class Run:
    def __init__(self, out: Path, env: dict[str, str]):
        self.out = out
        self.env = env
        self.processes: list[tuple[str, subprocess.Popen]] = []
        self.streams = []
        self.records = []

    def start(self, name: str, command: list[str]) -> subprocess.Popen:
        stream = (self.out / 'logs' / f'{name}.log').open('w')
        self.streams.append(stream)
        process = subprocess.Popen(command, env=self.env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append((name, process))
        self.records.append({'name': name, 'pid': process.pid, 'command': command})
        return process

    def alive(self) -> None:
        for name, process in self.processes:
            if process.poll() is not None:
                raise RuntimeError(f'{name} exited early: {process.returncode}; see logs/{name}.log')

    def pause(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.alive()
            time.sleep(min(.2, max(0., end - time.monotonic())))

    def stop(self) -> list[dict]:
        """SIGINT closes rosbag/MP4 first; escalation affects only our groups."""
        records = []
        for name, process in reversed(self.processes):
            escalation = None
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=10)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    escalation = 'SIGTERM'
                    try:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
                    except ProcessLookupError:
                        pass
                    except subprocess.TimeoutExpired:
                        escalation = 'SIGKILL'
                        try:
                            os.killpg(process.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                        process.wait(timeout=5)
            # A launch parent may have exited while children still own its group.
            # Its group is private to this runner; never signal another session.
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            records.append({'name': name, 'returncode': process.poll(),
                            'cleanup_escalation': escalation})
        for stream in self.streams:
            stream.close()
        return records


def main() -> int:
    args = arguments()
    out = args.out.expanduser().resolve()
    here = Path(__file__).resolve().parent
    monitor_path = here / 'monitor.py'
    # Native rclpy is an intentional guard against reusing the extracted runtime.
    import rclpy
    import yaml
    from ament_index_python.packages import get_package_share_directory
    rclpy_path = Path(rclpy.__file__).resolve()
    if not rclpy_path.is_relative_to(Path('/opt/ros/humble')):
        raise RuntimeError(f'Activate native ROS 2 first; rclpy currently comes from {rclpy_path}')
    for binary in ('ros2', 'xacro'):
        if shutil.which(binary) is None:
            raise RuntimeError(f'Required executable missing: {binary}')
    xvfb = shutil.which('Xvfb')
    if xvfb is None:
        local_xvfb = here.parents[2] / 'work/ros_root/usr/bin/Xvfb'
        if not local_xvfb.is_file():
            raise RuntimeError('Xvfb is required for the private recording display')
        xvfb = str(local_xvfb)
    if args.record_video and not Path('/usr/bin/ffmpeg').is_file():
        raise RuntimeError('/usr/bin/ffmpeg is required for video recording')
    share = Path(get_package_share_directory('f1tenth_gym_ros')).resolve()
    base_config = share / 'config/sim.yaml'
    config = yaml.safe_load(base_config.read_text())
    bridge = config['bridge']['ros__parameters']
    if bridge.get('num_agent') != 1:
        raise RuntimeError('This runner validates the individual, one-vehicle activity')
    bridge['map_path'] = str(args.map_stem.expanduser().resolve())
    map_yaml = Path(bridge['map_path'] + '.yaml')
    map_image = Path(bridge['map_path'] + bridge['map_img_ext'])
    for required in (monitor_path, map_yaml, map_image,
                     share / 'launch/gym_bridge.rviz', share / 'launch/ego_racecar.xacro'):
        if not required.is_file():
            raise FileNotFoundError(required)
    display_number = args.display[1:]
    for occupied in (Path(f'/tmp/.X11-unix/X{display_number}'),
                     Path(f'/tmp/.X{display_number}-lock')):
        if occupied.exists():
            raise RuntimeError(f'Display {args.display} is occupied ({occupied}); select another display')
    # mkdir(exist_ok=False) is the sole output reservation; no overwrite option.
    out.parent.mkdir(parents=True, exist_ok=True)
    out.mkdir(exist_ok=False)
    (out / 'logs').mkdir()
    (out / 'run').mkdir()
    (out / 'ros_logs').mkdir()
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(out / 'ros_logs'), DISPLAY=args.display,
               LIBGL_ALWAYS_SOFTWARE='1', QT_QPA_PLATFORM='xcb', PYTHONUNBUFFERED='1')
    env.setdefault('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')
    env.setdefault('NUMBA_CACHE_DIR', '/tmp/daniel_numba_fast')
    Path(env['NUMBA_CACHE_DIR']).mkdir(parents=True, exist_ok=True)
    bridge['validation_lockstep'] = args.mode == 'lockstep'
    per_run_config = out / 'bridge.yaml'
    per_run_config.write_text(yaml.safe_dump(config, sort_keys=False))
    launch_path = out / 'candidate.launch.py'
    launch_path.write_text(launch_source(per_run_config, share, map_yaml))
    controller_parameters = {'use_sim_time': True, 'target_laps': 10,
                             'controller_profile': args.controller_profile,
                             'max_speed': args.max_speed,
                             'csv_path': str(out / 'run/laps_ros.csv')}
    for key in ('gain', 'lateral_accel', 'brake_accel', 'smooth', 'alpha'):
        value = getattr(args, key)
        if value is not None:
            controller_parameters[key] = value
    command = ['ros2', 'run', 'daniel_gap', 'gap_node', '--ros-args']
    for name, value in controller_parameters.items():
        text = str(value).lower() if isinstance(value, bool) else str(value)
        command.extend(['-p', f'{name}:={text}'])
    sources = {}
    for module_name in ('rclpy', 'daniel_gap.ros_node', 'daniel_gap.core',
                        'daniel_gap.fast_core', 'f110_gym.envs.f110_env',
                        'f1tenth_gym_ros.gym_bridge'):
        module = importlib.import_module(module_name)
        source = Path(module.__file__).resolve()
        sources[module_name] = {'path': str(source), 'sha256': digest(source)}
    started_wall = time.monotonic()
    manifest = {
        'started_utc': datetime.now(timezone.utc).isoformat(),
        'mode': args.mode, 'controller_parameters': controller_parameters,
        'domain': args.domain, 'display': args.display, 'display_server': xvfb,
        'timeout_wall_seconds': args.timeout_wall,
        'monitor_builtin_timeout_seconds': 3600,
        'startup_seconds': args.startup_seconds,
        'native_python': sys.executable, 'native_modules': sources,
        'simulator_share': str(share), 'source_bridge_config': str(base_config),
        'source_bridge_config_sha256': digest(base_config),
        'map_yaml': str(map_yaml), 'map_yaml_sha256': digest(map_yaml),
        'map_image': str(map_image), 'map_image_sha256': digest(map_image),
        'bridge_overrides': {'validation_lockstep': bridge['validation_lockstep'],
                             'map_path': bridge['map_path']},
        'physics_overrides': {}, 'video_requested': args.record_video,
        'video': {'size': '1920x1080', 'fps': 15, 'codec': 'libx264',
                  'movflags': '+faststart'} if args.record_video else None,
        'status': 'starting', 'commands': [],
        'timing_note': 'Lap times use simulation clock; video duration uses wall time.',
    }
    manifest_path = out / 'manifest.json'
    def save_manifest():
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    save_manifest()
    run = Run(out, env)
    error = None
    interrupted = False
    completed = False
    observation = {'collision': False, 'invalid_status': False, 'latest_status': None}
    # Our small live observer aborts immediately on an invalid run. The separate
    # monitor.py remains the writer of the complete DDS evidence and summary.
    from std_msgs.msg import Bool, String
    os.environ.update({key: env[key] for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY',
                                                'ROS_LOG_DIR', 'RMW_IMPLEMENTATION')})
    rclpy.init()
    guard = rclpy.create_node('daniel_candidate_guard')
    def collision(msg):
        observation['collision'] |= bool(msg.data)
    def status(msg):
        observation['latest_status'] = msg.data
        observation['invalid_status'] |= 'Invalido True' in msg.data
    guard.create_subscription(Bool, '/daniel_gap/collision', collision, 10)
    guard.create_subscription(String, '/daniel_gap/status', status, 10)
    try:
        run.start('xvfb', [xvfb, args.display, '-screen', '0', '1920x1080x24',
                           '-ac', '-nolisten', 'tcp', '-noreset'])
        socket = Path(f'/tmp/.X11-unix/X{display_number}')
        ready_deadline = time.monotonic() + 10
        while not socket.exists():
            if time.monotonic() > ready_deadline:
                raise TimeoutError('Xvfb display did not become ready within 10 seconds')
            run.pause(.1)
        # Observer precedes infrastructure and controller for collision evidence.
        monitor = run.start('monitor', [sys.executable, str(monitor_path), str(out / 'run')])
        run.pause(1)
        run.start('infrastructure', ['ros2', 'launch', str(launch_path)])
        run.start('visual_hud', [sys.executable, str(here / 'follow_hud.py')])
        run.pause(args.startup_seconds)
        subprocess.run([sys.executable, str(here / 'close_private_docks.py'), args.display], env=env, check=True)
        run.pause(1)
        subprocess.run(['/usr/bin/ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                        '-f', 'x11grab', '-video_size', '1920x1080', '-i', args.display + '.0',
                        '-frames:v', '1', str(out / 'preflight.png')],
                       env=env, check=True, timeout=20)
        manifest.update(status='awaiting_preflight_review', commands=run.records)
        manifest['visualization'] = {'recording':'real RViz X11 capture', 'overview':'overview.rviz',
                                     'detail':'follow.rviz', 'detail_target_frame':'ego_racecar/base_link',
                                     'physics_control_timing_changes':False}
        save_manifest()
        print(f'READY_FOR_CONTROLLER preflight={out / "preflight.png"} start_file={args.start_file}', flush=True)
        while not args.start_file.is_file():
            if time.monotonic() - started_wall > args.timeout_wall:
                raise TimeoutError('Preflight review timed out')
            run.pause(.5)
        if args.record_video:
            run.start('capture', ['/usr/bin/ffmpeg', '-hide_banner', '-nostats',
                                 '-f', 'x11grab', '-framerate', '15',
                                 '-video_size', '1920x1080', '-i', args.display + '.0',
                                 '-an', '-c:v', 'libx264', '-preset', 'veryfast',
                                 '-crf', '23', '-pix_fmt', 'yuv420p',
                                 '-movflags', '+faststart',
                                 str(out / 'evidencia_ros2_rviz.mp4')])
            run.pause(1)
        run.start('controller', command)
        manifest.update(status='running', commands=run.records)
        save_manifest()
        print(f'RUN {out} | native ROS 2 | {args.mode} | max_speed={args.max_speed}', flush=True)
        progress_at = time.monotonic()
        while monitor.poll() is None:
            for _ in range(20):
                rclpy.spin_once(guard, timeout_sec=0.)
            if observation['collision'] or observation['invalid_status']:
                raise RuntimeError('Collision or invalid controller status; aborting candidate')
            if time.monotonic() - started_wall > args.timeout_wall:
                raise TimeoutError(f'Wall timeout after {args.timeout_wall:.1f} seconds')
            for name, process in run.processes:
                if name != 'monitor' and process.poll() is not None:
                    raise RuntimeError(f'{name} exited early: {process.returncode}; see logs/{name}.log')
            if time.monotonic() >= progress_at:
                csv_path = out / 'run/laps_ros.csv'
                lines = csv_path.read_text().splitlines()[1:] if csv_path.exists() else []
                print(f'PROGRESS wall={time.monotonic() - started_wall:.1f}s '
                      f'laps={len(lines)}/10 last={lines[-1] if lines else "pending"}', flush=True)
                progress_at = time.monotonic() + 30
            time.sleep(.5)
        summary_path = out / 'run/summary_ros.json'
        if not summary_path.is_file():
            raise RuntimeError('Monitor exited without a summary; see logs/monitor.log')
        summary = json.loads(summary_path.read_text())
        completed = (summary.get('completed') is True
                     and len(summary.get('laps', [])) == 10
                     and summary.get('collision') is False
                     and not observation['collision']
                     and not observation['invalid_status'])
        manifest['status'] = 'completed' if completed else 'incomplete'
        print(json.dumps(summary, indent=2), flush=True)
    except KeyboardInterrupt:
        interrupted = True
        manifest['status'] = 'interrupted'
        print('Interrupted; closing owned nodes, rosbag and MP4.', flush=True)
    except Exception as exc:
        error = repr(exc)
        manifest.update(status='failed', error=error)
        print(f'ERROR {error}', file=sys.stderr, flush=True)
    finally:
        manifest['commands'] = run.records
        manifest['process_exit'] = run.stop()
        manifest['ended_utc'] = datetime.now(timezone.utc).isoformat()
        manifest['wall_seconds'] = time.monotonic() - started_wall
        manifest['live_guard'] = observation
        summary_path = out / 'run/summary_ros.json'
        if summary_path.is_file():
            manifest['summary'] = json.loads(summary_path.read_text())
        video = out / 'evidencia_ros2_rviz.mp4'
        if video.is_file():
            manifest['video_bytes'] = video.stat().st_size
        guard.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        save_manifest()
    return 130 if interrupted else 1 if error else 0 if completed else 2


if __name__ == '__main__':
    raise SystemExit(main())
