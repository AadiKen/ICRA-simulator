# Vehicle B production integration

Vehicle B now executes through the production `MarineSimulation` Node path using the actual `coupled6` plant. Its planar hull, propeller, and rudder loads come from the selectable MMG force model. Heave, roll, and pitch remain dynamically free and use the resolved Capytaine added mass, potential-radiation damping, hydrostatic stiffness, and independently empirical viscous damping.

The production adapter removes generic surge, sway, and yaw damping when MMG hull forces are active, preventing double counting. It retains the coupled6 mass, Coriolis, hydrostatic, wind, and out-of-plane damping paths. Propeller and rudder lag/rate state is checkpointed and restores bit-exactly during a transient.

The production parameter set is `vehicle-b-usv-bootstrap`. It is independently defined at USV scale and contains no KVLCC2/KCS coefficient transfer. Its checksum and the resolved hydrodynamics checksum are included in every step's vehicle-path metadata.

This is integration evidence, not behavioral validation. The USV maneuvering coefficients, viscous damping, and bootstrap hull mesh still require independent Vehicle B measurements before physical-accuracy claims or benchmark acceptance.
