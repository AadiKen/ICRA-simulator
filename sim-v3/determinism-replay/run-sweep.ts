import {readFile, mkdir, writeFile} from "node:fs/promises";
import {join, resolve} from "node:path";
import {fileURLToPath} from "node:url";
import {bcodAdapter} from "./bcod-adapter.ts";
import {SIMULATORS, validateRoute, type RouteFixture, type Simulator} from "./types.ts";

const root = resolve(fileURLToPath(new URL("..", import.meta.url)));
const adapters = {"bcod-sim": bcodAdapter} as const;
function args() {
  const values: Record<string, string> = {simulator: "all", route: "determinism-replay-route-v1", n: "30", out: "determinism-replay"};
  for (let i = 2; i < process.argv.length; i += 2) {
    const key = process.argv[i];
    if (!key?.startsWith("--") || !process.argv[i + 1] || !(key.slice(2) in values)) throw Error(`Invalid argument ${key ?? ""}`);
    values[key.slice(2)] = process.argv[i + 1];
  }
  const n = Number(values.n);
  if (!Number.isInteger(n) || n < 2) throw Error("--n must be an integer >= 2");
  const selected = values.simulator === "all" ? [...SIMULATORS] : values.simulator.split(",");
  if (!selected.length || new Set(selected).size !== selected.length || selected.some(v => !SIMULATORS.includes(v as Simulator))) throw Error("Unknown or duplicate simulator");
  return {n, selected: selected as Simulator[], route: values.route, out: resolve(root, values.out)};
}
async function main() {
  const options = args();
  const route = JSON.parse(await readFile(join(root, "fixtures", `${options.route}.json`), "utf8")) as RouteFixture;
  validateRoute(route);
  // Fail before writing any logs: unavailable native arms must never become synthetic evidence.
  const unavailable = options.selected.filter(sim => !(sim in adapters));
  if (unavailable.length) throw Error(`No native Vehicle A Otter open-loop replay adapter for: ${unavailable.join(", ")}. Run --simulator bcod-sim to collect the supported arm; five-arm figure generation remains blocked.`);
  let crashes = 0;
  for (const simulator of options.selected) {
    const adapter = adapters[simulator as keyof typeof adapters];
    const directory = join(options.out, simulator, route.route_id);
    await mkdir(directory, {recursive: true});
    for (let replayIndex = 0; replayIndex < options.n; replayIndex++) {
      const log = await adapter.runReplay(route, replayIndex);
      if (log.simulator !== simulator || log.route_id !== route.route_id || log.replay_index !== replayIndex || log.vehicle !== route.vehicle) throw Error(`Adapter ${simulator} returned mismatched provenance`);
      await writeFile(join(directory, `run_${String(replayIndex).padStart(2, "0")}.json`), `${JSON.stringify(log, null, 2)}\n`);
      if (log.crashed_at_t !== undefined) crashes++;
      console.log(`${simulator} ${replayIndex + 1}/${options.n}${log.crashed_at_t === undefined ? "" : ` crashed at ${log.crashed_at_t}s`}`);
    }
  }
  if (crashes) throw Error(`${crashes} replay(s) crashed; partial logs retained`);
}
await main();
