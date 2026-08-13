import type {SensorSample} from "../runtime.ts";

export interface PublishedSensorReader{latest(pluginId:string):SensorSample|null}

/** Read-only facade: consumers receive clones, never registry or plugin references. */
export class PublishedSensorsService implements PublishedSensorReader{
  readonly #resolve:(pluginId:string)=>SensorSample|null;
  constructor(resolve:(pluginId:string)=>SensorSample|null){this.#resolve=resolve;}
  latest(pluginId:string):SensorSample|null{return structuredClone(this.#resolve(pluginId));}
}
