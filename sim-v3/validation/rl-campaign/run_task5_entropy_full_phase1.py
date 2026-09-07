#!/usr/bin/env python3
"""Full 1.5M Task 5 Phase-1 curve; intentionally never starts Phase 2."""
from __future__ import annotations
import json,time,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'packages/python-client'));sys.path.insert(0,str(Path(__file__).resolve().parent))
from bcod_sim import CommonWaypointEnv  # noqa:E402
from run_task5_conditions import evaluate,factory,atomic_json  # noqa:E402

OUT=ROOT/'artifacts/rl-campaign/surveyor/task-5-default-noise-ent-0p005-phase1-1p5m/seed-7319'
CHECKPOINTS=range(250_000,1_500_001,250_000)
V6=[.04,.12,.14,.24,.48,.50]
CONTRACT_PATH=ROOT/'artifacts/rl-campaign/surveyor/task-contract-frozen.json'
def execution_signature(document):
 task=next(x for x in document['tasks'] if x['task_id']=='common-waypoint-transit-v1');action={k:v for k,v in task['action'].items() if k!='fairness'}
 return json.dumps({k:task.get(k) for k in ('timing','observation','reward','termination','reset_randomization','waypoint_progression','vehicle')}|{'action':action},sort_keys=True,separators=(',',':'))
LAUNCH_CONTRACT=json.loads(CONTRACT_PATH.read_text());LAUNCH_EXECUTION_SIGNATURE=execution_signature(LAUNCH_CONTRACT)
def refresh_metadata_only_contract_change():
 current=json.loads(CONTRACT_PATH.read_text())
 if execution_signature(current)!=LAUNCH_EXECUTION_SIGNATURE:raise RuntimeError('Frozen contract execution fields changed during Phase 1; refusing to continue.')
 CommonWaypointEnv.EXPECTED_CONTRACT_SHA256=current['content_sha256']
 return current['content_sha256']

def plot(curve):
 w,h,l,t,r,b=760,430,65,35,25,55;sx=lambda i:l+i*(w-l-r)/5;sy=lambda v:t+(1-v)*(h-t-b)
 lines=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">','<rect width="100%" height="100%" fill="white"/>','<text x="380" y="22" text-anchor="middle" font-family="sans-serif" font-size="16">Phase-1 success curve: corrected Task 5 vs v6</text>',f'<line x1="{l}" y1="{t}" x2="{l}" y2="{h-b}" stroke="black"/>',f'<line x1="{l}" y1="{h-b}" x2="{w-r}" y2="{h-b}" stroke="black"/>']
 for y in (0,.25,.5,.75,1):lines += [f'<line x1="{l}" y1="{sy(y)}" x2="{w-r}" y2="{sy(y)}" stroke="#ddd"/>',f'<text x="{l-8}" y="{sy(y)+4}" text-anchor="end" font-family="sans-serif" font-size="11">{y:.2f}</text>']
 for i,step in enumerate(CHECKPOINTS):lines.append(f'<text x="{sx(i)}" y="{h-b+20}" text-anchor="middle" font-family="sans-serif" font-size="11">{step//1000}k</text>')
 for vals,color,name in ((V6,'#888','v6'),([x['evaluation']['success_rate'] for x in curve],'#2563eb','corrected + entropy')):
  pts=' '.join(f'{sx(i):.1f},{sy(v):.1f}' for i,v in enumerate(vals));lines.append(f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="3"/>');
  for i,v in enumerate(vals):lines.append(f'<circle cx="{sx(i)}" cy="{sy(v)}" r="4" fill="{color}"/>')
 lines += ['<text x="620" y="55" font-family="sans-serif" fill="#2563eb">corrected + entropy</text>','<text x="620" y="75" font-family="sans-serif" fill="#888">v6</text>','</svg>'];(OUT/'success-curve-vs-v6.svg').write_text('\n'.join(lines)+'\n')

def main():
 from sb3_contrib import RecurrentPPO
 from stable_baselines3.common.vec_env import SubprocVecEnv
 OUT.mkdir(parents=True,exist_ok=True);env=SubprocVecEnv([factory(i,'default-noise') for i in range(16)],start_method='fork')
 existing=json.loads((OUT/'phase-1-report.json').read_text()) if (OUT/'phase-1-report.json').exists() else None
 curve=existing.get('curve',[]) if existing else [];saved=[]
 for nominal in CHECKPOINTS:
  if (OUT/f'phase1-{nominal}.zip').exists():saved.append(nominal)
 latest=max(saved,default=0)
 if latest:model=RecurrentPPO.load(OUT/f'phase1-{latest}.zip',env=env,device='cpu')
 else:model=RecurrentPPO('MlpLstmPolicy',env,seed=7319,n_steps=512,batch_size=512,n_epochs=10,learning_rate=3e-4,gamma=.99,gae_lambda=.95,clip_range=.2,ent_coef=.005,policy_kwargs={'net_arch':[128,128],'lstm_hidden_size':128,'enable_critic_lstm':True},verbose=1,device='cpu')
 started=time.time();atomic_json(OUT/'training-state.json',{'status':'phase-1-running','nominal_timesteps':latest,'actual_model_timesteps':model.num_timesteps,'resumed_from_checkpoint':latest or None,'phase_2_started':False,'contract_sha256':CommonWaypointEnv.EXPECTED_CONTRACT_SHA256})
 try:
  for nominal in CHECKPOINTS:
   prior=next((x for x in curve if x['checkpoint_steps']==nominal),None)
   if nominal<=latest:
    if prior:continue
    refresh_metadata_only_contract_change();result=evaluate(model,'default-noise')
   else:
    model.learn(total_timesteps=250_000,reset_num_timesteps=False,progress_bar=False);model.save(OUT/f'phase1-{nominal}');refresh_metadata_only_contract_change();result=evaluate(model,'default-noise')
   row={'checkpoint_steps':nominal,'actual_model_timesteps':model.num_timesteps,'evaluation':result,'v6_success_rate':V6[list(CHECKPOINTS).index(nominal)]};curve.append(row);curve.sort(key=lambda x:x['checkpoint_steps']);current_hash=refresh_metadata_only_contract_change();atomic_json(OUT/'phase-1-report.json',{'schema_version':1,'artifact_kind':'task-5-default-noise-entropy-full-phase1','status':'running','contract_sha256':current_hash,'contract_transition':{'training_started_under':'b55b74e2485b3a2cd186779fb6d3efef6e466beaee639d6739170ff8321e97df','current':current_hash,'execution_fields_changed':False,'change':'Gate-5 reporting metadata only'},'training':{'seed':7319,'ent_coef':.005,'condition':'default-noise','algorithm':'RecurrentPPO','policy':'MlpLstmPolicy','nominal_budget':1_500_000},'v6_reference_curve':V6,'curve':curve,'cross_track_context':{'finding':'At 250k, failed policies accumulated median cross-track penalties roughly 5.5-8.0 times the magnitude of progress. Retained as diagnostic context; no reward term changed.'},'phase_2_started':False});atomic_json(OUT/'training-state.json',{'status':'phase-1-running','nominal_timesteps':nominal,'actual_model_timesteps':model.num_timesteps,'latest_success_rate':result['success_rate'],'phase_2_started':False});print(json.dumps({'checkpoint_steps':nominal,'success_rate':result['success_rate'],'median_shaped_return':result['median_return'],'median_progress':result['median_component_contributions']['progress'],'v6':V6[list(CHECKPOINTS).index(nominal)]}),flush=True)
 finally:env.close()
 plot(curve);report=json.loads((OUT/'phase-1-report.json').read_text());report['status']='complete-awaiting-review';report['wall_clock_s']=time.time()-started;report['phase_2_started']=False;atomic_json(OUT/'phase-1-report.json',report);atomic_json(OUT/'training-state.json',{'status':'phase-1-complete-awaiting-review','nominal_timesteps':1_500_000,'actual_model_timesteps':model.num_timesteps,'phase_2_started':False,'report':str((OUT/'phase-1-report.json').relative_to(ROOT))})
if __name__=='__main__':main()
