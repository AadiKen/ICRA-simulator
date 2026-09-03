import {readFileSync,writeFileSync} from "node:fs";

const path=process.argv[2];
if(!path)throw new Error("usage: apply-vrx-planar-constraint.ts model.sdf");
const source=readFileSync(path,"utf8");
const constraint=`
    <!-- Diagnostic-only physical joint chain: free x, y and yaw; locked z,
         roll and pitch. Carriage masses are negligible (1e-6 kg each). -->
    <link name="planar_x_carriage"><inertial><mass>1e-6</mass><inertia><ixx>1e-9</ixx><iyy>1e-9</iyy><izz>1e-9</izz></inertia></inertial></link>
    <link name="planar_y_carriage"><inertial><mass>1e-6</mass><inertia><ixx>1e-9</ixx><iyy>1e-9</iyy><izz>1e-9</izz></inertia></inertial></link>
    <joint name="planar_x" type="prismatic"><parent>world</parent><child>planar_x_carriage</child><axis><xyz expressed_in="__model__">1 0 0</xyz><limit><lower>-1e6</lower><upper>1e6</upper></limit></axis></joint>
    <joint name="planar_y" type="prismatic"><parent>planar_x_carriage</parent><child>planar_y_carriage</child><axis><xyz expressed_in="__model__">0 1 0</xyz><limit><lower>-1e6</lower><upper>1e6</upper></limit></axis></joint>
    <joint name="planar_yaw" type="revolute"><parent>planar_y_carriage</parent><child>base_link</child><axis><xyz expressed_in="__model__">0 0 1</xyz><limit><lower>-1e6</lower><upper>1e6</upper></limit></axis></joint>
`;
if(!source.includes('<link name="base_link">'))throw new Error("base_link insertion point missing");
writeFileSync(path,source.replace('<link name="base_link">',constraint+'    <link name="base_link">'));
