import {frozenReset} from "./episode-driver.ts";
const seeds=process.argv.slice(2).map(Number);
if(!seeds.length||seeds.some(seed=>!Number.isInteger(seed)))throw new Error("provide integer seeds");
console.log(JSON.stringify(seeds.map(seed=>frozenReset(seed))));
