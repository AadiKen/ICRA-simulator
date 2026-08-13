import {VehicleParameters} from "../vehicleParameters.js";
import {otterCoefficients} from "./coefficients.js";

export function createOtterParameters(overrides = {}) {
    return VehicleParameters.fromCoefficientSet(otterCoefficients, overrides);
}
