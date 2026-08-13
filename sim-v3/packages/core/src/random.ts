export interface SeededRandomState {algorithm: "mulberry32"; state_u32: number}

export class SeededRandom {
  #state: number;
  constructor(seed: number) { this.#state = (seed >>> 0); }
  next(): number {
    this.#state = (this.#state + 0x6D2B79F5) >>> 0;
    let value = this.#state;
    value = Math.imul(value ^ value >>> 15, value | 1);
    value ^= value + Math.imul(value ^ value >>> 7, value | 61);
    return ((value ^ value >>> 14) >>> 0) / 4294967296;
  }
  saveState(): SeededRandomState { return {algorithm: "mulberry32", state_u32: this.#state}; }
  loadState(saved: SeededRandomState): void {
    if (saved?.algorithm !== "mulberry32" || !Number.isInteger(saved.state_u32) || saved.state_u32 < 0 || saved.state_u32 > 0xffffffff) throw new Error("Invalid seeded RNG checkpoint state.");
    this.#state = saved.state_u32 >>> 0;
  }
}
