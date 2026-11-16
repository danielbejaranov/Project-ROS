"""Ensayo reproducible del motor oficial; guarda telemetría y resultados reales."""
from pathlib import Path
import csv, json, argparse
import numpy as np
# Compatibilidad Gym 0.19 con NumPy reciente.
if 'float' not in np.__dict__: np.float = float
import gym
from daniel_gap.core import GapController, LapCounter

ROOT = Path(__file__).resolve().parent

def main():
    parser=argparse.ArgumentParser(); parser.add_argument('--max-speed',type=float,default=2.5)
    parser.add_argument('--laps',type=int,default=10); args=parser.parse_args()
    env=gym.make('f110_gym:f110-v0',map=str(ROOT/'results/Sakhir_map'),map_ext='.png',num_agents=1)
    # Centro del tramo izquierdo, sentido hacia el sur de la imagen.
    start=np.array([[0.5, 15.0, -np.pi/2]])
    obs,_,_,_=env.reset(start)
    control=GapController(args.max_speed); counter=LapCounter(*start[0])
    rows=[]; t=0.; collision=False; stopped=0
    counter.update(*start[0,:2],t)
    for step in range(300000):
        steer,speed=control.command(obs['scans'][0],-2.35,4.7/(len(obs['scans'][0])-1))
        obs,dt,done,info=env.step(np.array([[steer,speed]])); t+=dt
        collision=bool(obs['collisions'][0])
        lap=counter.update(obs['poses_x'][0],obs['poses_y'][0],t,collision)
        if step%10==0:
            rows.append([t,obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],speed,steer,len(counter.times),t-counter.started])
        if step%5000==0: print('progress',step, t,obs['poses_x'][0],obs['poses_y'][0],speed,'invalid',counter.invalid,flush=True)
        if lap: print(f'Vuelta {len(counter.times)}: {lap:.2f} s',flush=True)
        # Gym marca done al llegar a dos vueltas; continuamos SOLO ese límite,
        # manteniendo el mismo estado físico y deteniendo ante cualquier colisión.
        stopped = stopped+1 if speed == 0 else 0
        if stopped > 100:
            np.savez(ROOT/'results/stopped.npz',scan=obs['scans'][0],pose=[obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0]])
            print('STOPPED',flush=True); break
        if collision or len(counter.times)>=args.laps: break
    rows.append([t,obs['poses_x'][0],obs['poses_y'][0],obs['poses_theta'][0],speed,steer,len(counter.times),t-counter.started])
    with (ROOT/'results/telemetry.csv').open('w') as f:
        writer=csv.writer(f); writer.writerow(['t','x','y','yaw','speed','steer','laps','lap_elapsed']); writer.writerows(rows)
    result={'student':'Daniel Bejarano','map':'Sakhir','engine':'f110_gym', 'start':start.tolist(),
            'max_speed':args.max_speed,'laps':counter.times,'collision':collision,'simulated_seconds':t,
            'completed':len(counter.times)>=args.laps and not collision,'ros2_tested':False}
    (ROOT/'results/summary.json').write_text(json.dumps(result,indent=2))
    print(result)
if __name__=='__main__': main()
