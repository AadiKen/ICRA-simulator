from __future__ import annotations
import unittest
import gymnasium as gym
import numpy as np
from .adapters import BcodSimAdapter,GazeboAdapter,HoloOceanAdapter,StonefishAdapter
from .adapters.base import AdapterConfigurationError,POLICY_OBSERVATION_FIELDS
from .adapters.determinism_gate import run_determinism_gate
from .calibrate_ceiling import confirm_all_arms,freeze_contract,select_candidate
from .contracts import load_contracts
from .judge.plant import AnalyticJudge
from .metrics import EpisodeMetrics,aggregate,wilson_interval
from .runner import ARMS,run_suite
from .preflight import gate2,gate3,gate4,verify_preflight
from .native_energy_cap import EnergyCapWrapper
class FakePolicy:
 def predict(self,observation,deterministic=True):return np.zeros(2,dtype=np.float32),None
class FakeNativeEnv(gym.Env):
 def __init__(self):self.control_interval_s=.1;self.contract={"evaluation_termination":{"energy_cap_ns":10.}};self.action_space=gym.spaces.Box(-1,1,(2,),np.float32);self.observation_space=gym.spaces.Box(-1,1,(1,),np.float32)
 def reset(self,*,seed=None,options=None):return np.zeros(1,np.float32),{}
 def step(self,action):return np.zeros(1,np.float32),0.,False,False,{"success":True,"termination_reason":"success","policy_propulsion_thrust_n":[30.,30.]}
class CommonSuiteTest(unittest.TestCase):
 def test_contract_hashes_and_shared_200_seeds(self):
  contracts=load_contracts();self.assertEqual({len(c.seeds) for c in contracts},{200});self.assertEqual(len({c.seeds for c in contracts}),1)
 def test_adapter_rejects_wrong_contract_width(self):
  with self.assertRaises(AdapterConfigurationError):GazeboAdapter(native_observation_keys=("one",))
 def test_all_adapters_match_exported_policy_contract(self):
  for adapter in (BcodSimAdapter(),GazeboAdapter(),HoloOceanAdapter(),StonefishAdapter()):self.assertEqual(adapter.native_obs_to_shared(np.zeros(15)).shape,(15,));self.assertEqual(adapter.native_action_to_shared([0,0]).shape,(2,))
 def test_adapter_gate_is_translation_only(self):
  adapter=GazeboAdapter();row={"observation":dict.fromkeys(POLICY_OBSERVATION_FIELDS,0.),"action":{"effector_0":.2,"effector_1":-.2}};self.assertTrue(run_determinism_gate(adapter,[row]).passed)
 def test_analytic_judge_single_step_and_observation(self):
  judge=AnalyticJudge();before=judge.state.time_s;judge.step([.5,.5]);self.assertAlmostEqual(judge.state.time_s-before,.05);self.assertEqual(judge.observation([10,0],[.5,.5],.9).shape,(15,))
 def test_metrics_group_by_arm_condition(self):
  rows=[EpisodeMetrics("gazebo","nominal","a"*64,seed,seed%2==0,float(seed),10,20,0,1000,50,.4) for seed in range(4)];cell=aggregate(rows,bootstrap_samples=100)[0];self.assertEqual(cell["success_rate"],.5);lo,hi=wilson_interval(2,4);self.assertLess(lo,.5);self.assertGreater(hi,.5)
 def test_runner_requires_preflight_authorization(self):
  adapters={"bcod-sim":BcodSimAdapter(),"gazebo":GazeboAdapter(),"holoocean":HoloOceanAdapter(),"stonefish":StonefishAdapter()}
  with self.assertRaisesRegex(RuntimeError,"preflight authorization"):run_suite(contracts=load_contracts(),adapters=adapters,policies={arm:FakePolicy() for arm in ARMS},training={arm:{"steps":1000,"wall_clock_s":1} for arm in ARMS},preflight={arm:{"passed":True} for arm in ARMS},judge_validation={"passed":True,"arms":[{"arm":arm,"domain_gap":.2} for arm in ARMS]},bootstrap_samples=10)
 def test_calibration_requires_all_unsaturated_arms(self):
  candidates=[{"success_radius_m":1.5},{"success_radius_m":2.}];selected,_=select_candidate(candidates,lambda item:.6 if item["success_radius_m"]==1.5 else .9);confirm_all_arms({arm:.5 for arm in ARMS});self.assertEqual(freeze_contract({"condition_id":"nominal","episode_seeds":[1]},selected)["status"],"frozen")
  with self.assertRaises(RuntimeError):confirm_all_arms({arm:(1. if arm=="gazebo" else .5) for arm in ARMS})
 def test_judge_gate_requires_human_review(self):
  rows=[{"arm":arm,"classification":"clean"} for arm in ARMS];self.assertFalse(gate2({"arms":rows,"human_reviewed":False})["passed"]);self.assertTrue(gate2({"arms":rows,"human_reviewed":True})["passed"])
 def test_adapter_gate_reports_every_arm(self):
  result=gate3({arm:{"passed":arm!="gazebo","status":"passed" if arm!="gazebo" else "failed"} for arm in ARMS});self.assertFalse(result["passed"]);self.assertEqual(len(result["adapters"]),4)
 def test_step_parity_uses_one_percent_when_cadences_differ(self):
  values={"target_steps":1_000_000,**{arm:{"steps":1_000_000,"checkpoint_save_interval":4096 if arm=="gazebo" else 250_000} for arm in ARMS}};result=gate4(values);self.assertTrue(result["passed"]);self.assertIn("1%",result["tolerance_rule"])
 def test_tampered_preflight_is_rejected(self):
  with self.assertRaisesRegex(RuntimeError,"hash mismatch"):verify_preflight({"content_sha256":"bad","passed":True,"runner_authorized":True,"gates":[]})
 def test_shared_energy_cap_uses_policy_only_thrust(self):
  env=EnergyCapWrapper(FakeNativeEnv());env.reset();*_,first=env.step([0,0]);*_,second=env.step([0,0]);self.assertEqual(first["propulsion_impulse_ns"],6.);self.assertFalse(first["energy_cap_exceeded"]);self.assertEqual(second["propulsion_impulse_ns"],12.);self.assertTrue(second["energy_cap_exceeded"]);self.assertFalse(second["success"])
if __name__=="__main__":unittest.main()
