import {ForceModel} from "./forceModel.js";

/**
 * MSS-compatible horizontal strip-theory cross-flow drag.
 * Mirrors LIBRARY/modeling/crossFlowDrag.m for sway and yaw.
 */
export class CrossFlowDrag extends ForceModel {
    computeWrench(ctx) {
        const config = ctx.params.damping?.crossFlow;
        if (!config?.enabled) return [0, 0, 0];

        const length = config.length ?? ctx.params.geometry?.length ?? 0;
        const draft = config.draft ?? ctx.params.geometry?.draft ?? 0;
        const cd = config.coefficient ?? 0;
        const rho = config.waterDensity ?? ctx.params.restoring?.waterDensity ?? 1025;
        const strips = config.strips ?? 20;
        const dx = length / strips;
        const v = ctx.relativeVelocityVector[1];
        const r = ctx.relativeVelocityVector[2];
        let sway = 0;
        let yaw = 0;

        // MSS includes both endpoints: -L/2:dx:L/2 (strips + 1 samples).
        for (let i = 0; i <= strips; i += 1) {
            const x = -length / 2 + i * dx;
            const local = v + x * r;
            const elemental = -0.5 * rho * draft * cd * Math.abs(local) * local * dx;
            sway += elemental;
            yaw += x * elemental;
        }
        return [0, sway, yaw];
    }
}
