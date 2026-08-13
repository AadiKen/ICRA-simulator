import {ForceModel} from "./forceModel.js";

export class RestoringForces extends ForceModel {
    computeWrench() {
        return [0, 0, 0];
    }
}
