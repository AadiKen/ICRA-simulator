import {nedBodyToWorld2D, nedWorldToBody2D} from "./frames.js";
import {matVecMul} from "./math.js";
import {stepRK4, stepSemiImplicitEuler} from "./integrator.js";
import {currentNedFromEnv} from "../packages/core/src/environment-forces.js";

export class DynamicsCore {
    constructor(params, forceModels = [], integrator = "rk4") {
        this.params = params;
        this.forceModels = forceModels;
        this.integrator = integrator;
        this.lastDerivative = [0, 0, 0, 0, 0, 0];
        this.lastWrench = [0, 0, 0];
    }

    step(state, env, command, dt, t = 0) {
        const stepper = this.integrator === "semiImplicitEuler"
            ? stepSemiImplicitEuler
            : stepRK4;
        const preparationContext = this.createContext(state, env, command, t, dt);
        this.forceModels.forEach((model) => {
            if (typeof model.prepareStep === "function") {
                model.prepareStep(preparationContext);
            }
        });
        stepper(this, state, env, command, dt, t);
        const derivative = this.derivative(state, env, command, t + dt);
        this.lastDerivative = derivative;
        state.acceleration = {uDot: derivative[3], vDot: derivative[4], wDot: 0};
        state.angularAccel = {pDot: 0, qDot: 0, rDot: derivative[5]};
        return state;
    }

    derivative(state, env, command, t = 0) {
        const ctx = this.createContext(state, env, command, t, 0);
        const wrench = this.forceModels.reduce((sum, model) => {
            const contribution = model.computeWrench(ctx);
            return [
                sum[0] + contribution[0],
                sum[1] + contribution[1],
                sum[2] + contribution[2]
            ];
        }, [0, 0, 0]);
        this.lastWrench = wrench;
        const nuDot = matVecMul(this.params.massMatrixInv, wrench);
        const etaDot = nedBodyToWorld2D(state.velocity.u, state.velocity.v, state.eulerAngles.yaw);

        return [
            etaDot.N,
            etaDot.E,
            state.angularRate.r,
            nuDot[0],
            nuDot[1],
            nuDot[2]
        ];
    }

    createContext(state, env, command, t = 0, dt = 0) {
        const yaw = state.eulerAngles.yaw;
        const currentNed = currentNedFromEnv(env);
        const currentBody = nedWorldToBody2D(currentNed.N, currentNed.E, yaw);
        const velocityVector = [state.velocity.u, state.velocity.v, state.angularRate.r];
        const relativeVelocityVector = [
            state.velocity.u - currentBody.u,
            state.velocity.v - currentBody.v,
            state.angularRate.r
        ];
        return {
            state,
            params: this.params,
            env,
            command,
            t,
            dt,
            currentBody,
            velocityVector,
            relativeVelocityVector
        };
    }
}
