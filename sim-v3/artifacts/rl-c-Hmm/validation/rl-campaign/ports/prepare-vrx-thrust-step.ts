import { readFileSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";
import { prepareVrxEpisode } from "./prepare-vrx-episode.ts";
import {
  FrozenActuatorBank,
  resolveSurveyorActuatorSpec,
  type FrozenAction,
} from "./shared-actuators.ts";

const out = resolve(process.argv[2] ?? "artifacts/rl-campaign/vrx-thrust-step");
const samples = 80,
  dt = 0.05,
  normalized = 0.25,
  targetNewtons = 17.5;
prepareVrxEpisode(20000, out, samples, 0, 1, 0, 0);
const schedulePath = resolve(out, "transport.json"),
  schedule = JSON.parse(readFileSync(schedulePath, "utf8"));
const bank = new FrozenActuatorBank(resolveSurveyorActuatorSpec()),
  action = [normalized, normalized, 0, 0] as FrozenAction;
schedule.schedule_kind = "static-thrust-step";
schedule.environment_scale = 0;
schedule.action_scale = 1;
schedule.step_target_newtons_each = targetNewtons;
schedule.action_trace = Array.from({ length: samples / 2 }, (_, index) => ({
  control_step: index,
  time_s: index * 0.1,
  command: action,
}));
schedule.transport = Array.from({ length: samples }, () => {
  bank.step(action, dt);
  const [port, starboard] = bank.thrustNewtons();
  return [
    {
      topic: "/surveyor/thrusters/port/thrust",
      message_type: "std_msgs/msg/Float64",
      value: port,
    },
    {
      topic: "/surveyor/thrusters/starboard/thrust",
      message_type: "std_msgs/msg/Float64",
      value: starboard,
    },
  ];
});
writeFileSync(schedulePath, JSON.stringify(schedule, null, 2) + "\n");
