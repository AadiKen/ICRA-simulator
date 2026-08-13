import {VehicleParameters} from "../vehicleParameters.js";
import {bcodUsvCoefficients} from "./coefficients.js";

export function createBcodUsvParameters(options = {}) {
    if (Object.keys(options).length === 0) {
        return VehicleParameters.fromCoefficientSet(bcodUsvCoefficients);
    }
    return VehicleParameters.fromCoefficientSet(bcodUsvCoefficients, {
        geometry: {
            length: options.length || bcodUsvCoefficients.geometry.length,
            beam: options.beam || bcodUsvCoefficients.geometry.beam,
            draft: options.draft || bcodUsvCoefficients.geometry.draft
        },
        massProps: {
            mass: options.mass || bcodUsvCoefficients.massProps.mass
        },
        actuator: {
            maxThrust: options.maxThrust || bcodUsvCoefficients.actuator.maxThrust,
            motorTimeConstant: options.motorTimeConstant || bcodUsvCoefficients.actuator.motorTimeConstant
        }
    });
}
