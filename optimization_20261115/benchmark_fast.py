"""Ensayo Gym reproducible del controlador FTG rápido, sin modificar el modelo."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument('--name', help='Nombre de un directorio nuevo en gym_runs/')
    destination.add_argument('--out', type=Path, help='Directorio nuevo de salida; nunca se sobrescribe')
    parser.add_argument('--vmax', type=float, default=20.)
    parser.add_argument('--gain', type=float, default=.7)
    parser.add_argument('--lat', type=float, default=12.5)
    parser.add_argument('--brake', type=float, default=8.25)
    parser.add_argument('--smooth', type=int, default=31)
    parser.add_argument('--alpha', type=float, default=.6)
    parser.add_argument('--laps', type=int, default=10)
    parser.add_argument('--settle-seconds', type=float, default=4.,
                        help='Tiempo físico adicional con velocidad cero tras completar las vueltas')
    parser.add_argument('--float32-scan', action='store_true')
    parser.add_argument('--ros-float32', action='store_true',
                        help='Cuantiza rangos, ángulos y órdenes como los campos ROS float32')
    args = parser.parse_args()
    if args.name is not None and (not args.name or Path(args.name).name != args.name or args.name in ('.', '..')):
        parser.error('--name debe ser un nombre simple de directorio')
    if not (0 < args.vmax <= 20 and args.gain > 0 and args.lat > 0 and args.brake > 0
            and args.smooth >= 1 and 0 < args.alpha <= 1 and args.laps >= 1
            and args.settle_seconds >= 0):
        parser.error('Parámetros fuera de rango; vmax debe estar en (0,20] y alpha en (0,1]')
    target = args.out.resolve() if args.out else HERE / 'gym_runs' / args.name
    try:
        target.mkdir(parents=True, exist_ok=False)
    except FileExistsError:
        parser.error(f'El directorio ya existe; elige una salida nueva: {target}')

    # Un caché local permite usar el simulador desde una instalación de solo lectura.
    os.environ.setdefault('NUMBA_CACHE_DIR', str(HERE / '.numba_cache'))
    sys.path.insert(0, str(ROOT))
    if 'float' not in np.__dict__:
        np.float = float  # Compatibilidad del motor Gym 0.19.
    import gym
    from daniel_gap.core import LapCounter
    from daniel_gap.fast_core import FastGapController

    env = gym.make('f110_gym:f110-v0', map=str(ROOT / 'results/Sakhir_map'),
                   map_ext='.png', num_agents=1, seed=12345, timestep=.01)
    start = np.array([[.5, 15., -np.pi / 2]])
    obs, _, _, _ = env.reset(start)
    controller = FastGapController(max_speed=args.vmax, gain=args.gain,
                                   lateral_accel=args.lat, brake_accel=args.brake,
                                   smooth=args.smooth, alpha=args.alpha)
    counter = LapCounter(*start[0])
    counter.update(*start[0, :2], 0.)
    t = 0.
    wall = time.monotonic()
    stopped = 0
    collision = False
    reason = 'max_steps'
    rows = []
    max_actual = 0.
    max_command = 0.
    finish_t = None
    for step in range(200000):
        scan = obs['scans'][0]
        if args.float32_scan or args.ros_float32:
            scan = scan.astype(np.float32)
        angle_min, increment = -2.35, 4.7 / (len(scan) - 1)
        if args.ros_float32:
            angle_min = float(np.float32(angle_min))
            increment = float(np.float32(increment))
        steering, speed = controller.command(scan, angle_min, increment)
        if finish_t is not None:
            speed = 0.  # Seguir guiando durante el frenado; no enderezar bruscamente.
        if args.ros_float32:
            steering, speed = float(np.float32(steering)), float(np.float32(speed))
        obs, dt, done, info = env.step(np.array([[steering, speed]]))
        t += dt
        collision = bool(obs['collisions'][0])
        max_actual = max(max_actual, float(obs['linear_vels_x'][0]))
        max_command = max(max_command, speed)
        lap = counter.update(obs['poses_x'][0], obs['poses_y'][0], t, collision)
        if step % 100 == 0:
            rows.append([t, float(obs['poses_x'][0]), float(obs['poses_y'][0]),
                         speed, steering, len(counter.times)])
        if lap:
            print(f'Vuelta {len(counter.times)}: {lap:.2f} s', flush=True)
        if step % 5000 == 0:
            print(f'Paso {step}: {t:.2f} s, velocidad ordenada {speed:.2f} m/s', flush=True)
        stopped = stopped + 1 if speed < .05 else 0
        # Gym finaliza convencionalmente a las dos vueltas. Se conserva su estado
        # físico para llegar a diez; cualquier colisión se revisa en cada paso.
        if collision:
            reason = 'collision'
            break
        if counter.invalid:
            reason = 'invalid_lap_counter'
            break
        if stopped >= 100 and finish_t is None:
            reason = 'stopped'
            break
        if len(counter.times) >= args.laps and finish_t is None:
            finish_t = t
            print(f'Meta alcanzada; comprobando {args.settle_seconds:.2f} s de frenado físico', flush=True)
        if finish_t is not None and t - finish_t >= args.settle_seconds - 1e-9:
            reason = 'completed' if abs(float(obs['linear_vels_x'][0])) < .05 else 'failed_to_stop'
            break

    parameters = vars(args).copy()
    parameters['out'] = str(args.out) if args.out else None
    result = {
        'name': args.name or target.name, 'args': parameters,
        'laps': counter.times, 'completed': reason == 'completed',
        'collision': collision, 'reason': reason,
        'collision_samples_checked': step + 1, 'simulated_seconds': t,
        'lap_total': finish_t, 'sim_total': t,
        'settle_seconds_executed': t - finish_t if finish_t is not None else 0.,
        'final_speed_m_s': float(obs['linear_vels_x'][0]),
        'wall_seconds': time.monotonic() - wall,
        'max_actual_speed': max_actual, 'max_command_speed': max_command,
        'seed': 12345, 'dt': .01, 'start': start.tolist(),
        'model_modified': False, 'map_modified': False,
        'physics_params': env.params,
        'controller_sha256': sha256(ROOT / 'daniel_gap/fast_core.py'),
        'lap_counter_and_inflation_sha256': sha256(ROOT / 'daniel_gap/core.py'),
        'map_sha256': sha256(ROOT / 'results/Sakhir_map.png'),
        'map_yaml_sha256': sha256(ROOT / 'results/Sakhir_map.yaml'),
        'runner_sha256': sha256(Path(__file__).resolve()),
        'numpy_version': np.__version__, 'gym_version': gym.__version__,
        'ros2_tested': False,
    }
    (target / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    np.savetxt(target / 'telemetry.csv', rows, delimiter=',',
               header='t,x,y,speed,steering,laps', comments='')
    if collision or reason == 'stopped':
        np.savez(target / 'stop.npz', scan=obs['scans'][0],
                 pose=[obs['poses_x'][0], obs['poses_y'][0], obs['poses_theta'][0]])
    env.close()
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
