// Compatibility facade retained only until the post-migration legacy API removal.
// The behavior-preserving implementation lives in the typed @bcod/core package.
export {
    ActuationModel,
    ActuatorModel,
    FixedThruster,
    AzimuthThruster,
    ControlSurface,
    Rotor,
    allocationMatrix,
    allocateLeastSquares,
    buildEffectors
} from "../../packages/core/src/actuators.js";
