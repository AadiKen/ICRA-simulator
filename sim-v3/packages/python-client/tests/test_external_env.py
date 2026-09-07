import json
from pathlib import Path
import sys
import tempfile
import unittest

from bcod_sim.external_env import ExternalRuntimeError, JsonLineSimulatorBridge, _assert_training_eligible


WORKER = r'''import json,sys
truth={"position_ned_m":[1,2,0],"attitude_rad":[0,0,.1],"velocity_body_mps":[.2,0,0],"acceleration_body_mps2":[0,0,0],"angular_rate_body_rad_s":[0,0,0]}
for line in sys.stdin:
 request=json.loads(line); op=request["op"]
 if op=="close": print(json.dumps({"ok":True}));sys.stdout.flush();break
 print(json.dumps({"ok":True,"truth":truth,"observations":[{"time_s":.1,"sensors":{}}],"terminated":False,"truncated":False,"info":{}}));sys.stdout.flush()
'''


class ExternalEnvTest(unittest.TestCase):
    def test_json_line_runtime_bridge(self):
        bridge = JsonLineSimulatorBridge([sys.executable, "-u", "-c", WORKER])
        bridge.reset([{"seed": 1}])
        self.assertEqual(bridge.ground_truth()["position_ned_m"][:2], [1, 2])
        result = bridge.step([{"actuators": {}}])
        self.assertEqual(result["terminated"], [False])
        self.assertEqual(result["observations"][0]["time_s"], .1)
        bridge.close()

    def test_conformance_guard_rejects_missing_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ExternalRuntimeError, "Gate 7"):
                _assert_training_eligible(Path(directory), "vrx")


if __name__ == "__main__":
    unittest.main()
