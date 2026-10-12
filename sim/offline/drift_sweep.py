import sys, math, numpy as np
REPO='/mnt/user-data/outputs/lostlink-sar'
sys.path.insert(0,REPO+'/sim/offline'); sys.argv=['x']
import offline_sim as S
from lostlink_planner import core
w=S.load_world(); world=S.World(w); wps=[(n,e) for (e,n) in w.WAYPOINTS]
def run(drift_pct, seed, gps_off_east=14.0):
    rng=np.random.default_rng(seed)
    proj=core.DepthProjector(stride=8); mf=core.MissionFollower(wps)
    pt=np.array([0,0,-3.0]); yaw=0.0; d=np.zeros(2); t=0; hits=[]; last=None
    ang=rng.uniform(0,2*np.pi); gps=True; traj=[]
    while t<300 and not mf.done:
        if gps and pt[1]>gps_off_east: gps=False
        pe=pt.copy(); pe[:2]+=d
        img,q=S.render_depth(world,pt,yaw,proj)
        mf.grid.insert(proj.to_world(img,pe,q))
        n,e,ys,_=mf.step(pe,yaw)
        v=0.95*(np.array([n,e])-pe[:2]); s=np.linalg.norm(v)
        if s>2: v*=2/s
        pt[:2]+=v*0.1; yaw+=np.clip(core.wrap(ys-yaw),-0.105,0.105)
        if not gps:
            ang+=rng.normal(0,0.05); d+=drift_pct/100*np.linalg.norm(v*0.1)*np.array([math.cos(ang),math.sin(ang)])
        h=world.collides(pt)
        if h and h!=last: hits.append(h)
        last=h; t+=0.1; traj.append((pt[0],pt[1],pe[0],pe[1]))
    np.save(f'/tmp/claude-0/s3/traj_{drift_pct}_{seed}.npy', np.array(traj))
    return mf.done, len(hits), sorted(set(hits)), round(float(np.linalg.norm(d)),2), np.round(pt[:2],2).tolist()
for dp in [float(x) for x in sys.argv_dp.split(',')] if hasattr(sys,'argv_dp') else [2,5,10,20]:
    for s in range(3):
        r=run(dp,s); print(dp, s, r, flush=True)
