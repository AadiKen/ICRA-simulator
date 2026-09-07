# Gazebo Harmonic 8.15 vehicle-plugin reference

Source image: `icra27-gazebo-harmonic:harmonic-8.15.0` (Gazebo Sim 8.15.0).
The XML below is verbatim from `/usr/share/gz/gz-sim8/worlds/auv_controls.sdf` in that image.

```xml
<plugin
  filename="gz-sim-joint-position-controller-system"
  name="gz::sim::systems::JointPositionController">
  <joint_name>horizontal_fins_joint</joint_name>
  <p_gain>0.1</p_gain>
</plugin>

<plugin
  filename="gz-sim-thruster-system"
  name="gz::sim::systems::Thruster">
  <namespace>tethys</namespace>
  <joint_name>propeller_joint</joint_name>
  <thrust_coefficient>0.004422</thrust_coefficient>
  <fluid_density>1000</fluid_density>
  <propeller_diameter>0.2</propeller_diameter>
</plugin>

<plugin
  filename="gz-sim-lift-drag-system"
  name="gz::sim::systems::LiftDrag">
  <air_density>1000</air_density>
  <cla>4.13</cla>
  <cla_stall>-1.1</cla_stall>
  <cda>0.2</cda>
  <cda_stall>0.03</cda_stall>
  <alpha_stall>0.17</alpha_stall>
  <a0>0</a0>
  <area>0.0244</area>
  <upward>0 1 0</upward>
  <forward>1 0 0</forward>
  <link_name>vertical_fins</link_name>
  <cp>0 0 0</cp>
</plugin>

<plugin
  filename="gz-sim-hydrodynamics-system"
  name="gz::sim::systems::Hydrodynamics">
  <link_name>base_link</link_name>
  <xDotU>-4.876161</xDotU>
  <yDotV>-126.324739</yDotV>
  <zDotW>-126.324739</zDotW>
  <kDotP>0</kDotP>
  <mDotQ>-33.46</mDotQ>
  <nDotR>-33.46</nDotR>
  <xUabsU>-6.2282</xUabsU>
  <xU>0</xU>
  <yVabsV>-601.27</yVabsV>
  <yV>0</yV>
  <zWabsW>-601.27</zWabsW>
  <zW>0</zW>
  <kPabsP>-0.1916</kPabsP>
  <kP>0</kP>
  <mQabsQ>-632.698957</mQabsQ>
  <mQ>0</mQ>
  <nRabsR>-632.698957</nRabsR>
  <nR>0</nR>
</plugin>
```

Parameter types used by Vehicle B, checked by loading the model with the 8.15
plugin binaries: names (`namespace`, `joint_name`, `link_name`), scalar
floating-point values (`p_gain`, `thrust_coefficient`, `fluid_density`,
`propeller_diameter`, all lift/drag coefficients and areas, and all added-mass
and damping terms), and three-component vectors (`upward`, `forward`, `cp`).
The 8.15 Thruster binary also contains `wake_fraction`, `alpha_1`, and
`alpha_2`. Vehicle B uses an explicit `thrust_coefficient`, for which 8.15 logs
that `alpha_1` and `alpha_2` are ignored, so they are deliberately absent from
the model rather than silently misapplied.

The pinned image contains `auv_controls.sdf` and `lrauv_control_demo.sdf`, but
does not contain a WAM-V model SDF. Consequently no WAM-V joint hierarchy can
truthfully be claimed as extracted from this image. The official surface-vessel
tutorial identifies the hierarchy `base_link -> left_chasis_engine_joint ->
engine -> propeller`; that external structural reference is recorded here only
for the gated Vehicle C task, which has not started.

