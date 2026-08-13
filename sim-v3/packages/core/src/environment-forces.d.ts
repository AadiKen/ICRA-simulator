export interface CurrentNed{N:number;E:number;D:number}
export declare function currentNedFromEnv(envSample?:unknown):CurrentNed;
export declare class WindLoad{config:Record<string,unknown>;lastFullWrench:[number,number,number,number,number,number];constructor(config?:Record<string,unknown>);computeWrench(ctx:Record<string,any>):[number,number,number]}
