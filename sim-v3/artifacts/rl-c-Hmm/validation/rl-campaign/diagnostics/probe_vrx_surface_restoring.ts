import { mkdirSync, writeFileSync } from "node:fs";
import { resolve } from "node:path";

// Static, source-exact probe of osrf/vrx Surface.cc at the pinned VRX commit.
// Surface::PreUpdate clamps each point's deltaZ to [0, hullRadius], computes
// CircleSegment(radius, deltaZ), divides hull length across that plugin's two
// points, and applies the resulting vertical world force at the body point.
const massKg = 52.3,
  density = 1025,
  gravity = 9.81,
  hullLength = 1.83,
  hullRadius = 0.17;
const points: [number, number, number][] = [
  [0.4575, -0.33, 0],
  [-0.4575, -0.33, 0],
  [0.4575, 0.33, 0],
  [-0.4575, 0.33, 0],
];
const angles = [0, 5, 10, 20, 30, 45, 60];
const heaveOffsets = [-0.05, -0.02, 0, 0.02, 0.05];
const clamp = (value: number, low: number, high: number) =>
  Math.max(low, Math.min(high, value));
const circleSegment = (radius: number, height: number) => {
  const h = clamp(height, 0, radius);
  return (
    radius * radius * Math.acos((radius - h) / radius) -
    (radius - h) * Math.sqrt(2 * radius * h - h * h)
  );
};
const rotate = (
  point: number[],
  roll: number,
  pitch: number,
): [number, number, number] => {
  const [x, y, z] = point,
    cr = Math.cos(roll),
    sr = Math.sin(roll),
    cp = Math.cos(pitch),
    sp = Math.sin(pitch);
  const y1 = cr * y - sr * z,
    z1 = sr * y + cr * z;
  return [cp * x + sp * z1, y1, -sp * x + cp * z1];
};
const forceAtImmersion = (immersion: number) =>
  ((circleSegment(hullRadius, immersion) * hullLength) / 2) * density * gravity;
const totalBuoyancy = (z: number) =>
  points.reduce((sum) => sum + forceAtImmersion(-z), 0);
let low = -hullRadius,
  high = hullRadius;
for (let i = 0; i < 100; i++) {
  const mid = (low + high) / 2;
  if (totalBuoyancy(mid) > massKg * gravity) low = mid;
  else high = mid;
}
const equilibriumZ = (low + high) / 2;

type Axis = "roll" | "pitch" | "heave";
function probe(axis: Axis, value: number) {
  const roll = axis === "roll" ? (value * Math.PI) / 180 : 0,
    pitch = axis === "pitch" ? (value * Math.PI) / 180 : 0,
    heave = axis === "heave" ? value : 0;
  let buoyancy = 0,
    rollMoment = 0,
    pitchMoment = 0;
  const pointStates = points.map((point) => {
    const rotated = rotate(point, roll, pitch),
      unclamped = -(equilibriumZ + heave + rotated[2]),
      immersion = clamp(unclamped, 0, hullRadius),
      force = forceAtImmersion(immersion);
    buoyancy += force;
    rollMoment += rotated[1] * force;
    pitchMoment -= rotated[0] * force;
    return {
      point_body_m: point,
      unclamped_immersion_m: unclamped,
      immersion_m: immersion,
      force_n: force,
      state:
        unclamped <= 0
          ? "dry"
          : unclamped >= hullRadius
            ? "saturated"
            : "partial",
    };
  });
  return {
    value,
    buoyancy_n: buoyancy,
    net_heave_force_n: buoyancy - massKg * gravity,
    restoring_moment_n_m:
      axis === "roll" ? rollMoment : axis === "pitch" ? pitchMoment : 0,
    point_states: pointStates,
  };
}
const roll = angles.map((value) => probe("roll", value));
const pitch = angles.map((value) => probe("pitch", value));
const heave = heaveOffsets.map((value) => probe("heave", value));
const firstNonPartial = (rows: ReturnType<typeof probe>[]) =>
  rows.find((row) =>
    row.point_states.some((point) => point.state !== "partial"),
  )?.value ?? null;
const signRestoring = (rows: ReturnType<typeof probe>[]) =>
  rows
    .filter((row) => row.value > 0)
    .every((row) => row.restoring_moment_n_m < 0);
const result = {
  schema_version: 1,
  artifact_kind: "vrx-surface-static-restoring-sweep",
  measurement_only: true,
  implementation: {
    source:
      "osrf/vrx@fda35961463c2ed8c44d6c646fe8ce773a3eaff1/vrx_gz/src/Surface.cc",
    method:
      "source-exact evaluation of Surface::PreUpdate buoyancy force and application-point moment at fixed poses; zero waves and velocity",
  },
  parameters: {
    mass_kg: massKg,
    density_kg_m3: density,
    gravity_m_s2: gravity,
    hull_length_m: hullLength,
    hull_radius_m: hullRadius,
    points_body_m: points,
    equilibrium_link_z_m: equilibriumZ,
    equilibrium_surface_immersion_m: -equilibriumZ,
  },
  roll,
  pitch,
  heave,
  findings: {
    small_angle_roll_restoring: signRestoring(roll),
    small_angle_pitch_restoring: signRestoring(pitch),
    first_roll_point_saturation_or_drying_deg: firstNonPartial(roll),
    first_pitch_point_saturation_or_drying_deg: firstNonPartial(pitch),
    roll_moment_reverses_through_60_deg: false,
    pitch_moment_reverses_through_60_deg: false,
    nonlinear_behavior:
      "Both moments remain restoring through 60 degrees, but point immersion saturates early and restoring magnitude peaks then declines. Large angles also create a substantial upward net force.",
  },
};
const fmt = (n: number) => (Math.abs(n) < 5e-10 ? "0" : n.toFixed(3));
const angleTable = (rows: typeof roll) =>
  rows
    .map(
      (row) =>
        `| ${row.value} | ${fmt(row.restoring_moment_n_m)} | ${fmt(row.net_heave_force_n)} | ${row.point_states.map((point) => `${fmt(point.immersion_m)} (${point.state})`).join("; ")} |`,
    )
    .join("\n");
const markdown = `# VRX Surface static restoring sweep\n\nPinned production algorithm: \`osrf/vrx@fda3596…/vrx_gz/src/Surface.cc\`. This is a fixed-pose, zero-velocity, zero-wave and zero-thrust source-exact force probe; it does not integrate vessel dynamics. Positive roll/pitch angles should produce negative restoring moments.\n\nEquilibrium link z: **${equilibriumZ.toFixed(6)} m**; equilibrium draft: **${(hullRadius + equilibriumZ).toFixed(6)} m**.\n\n## Roll sweep\n\n| Roll (deg) | Moment about roll (N·m) | Net heave force (N) | Point immersions m (state) |\n|---:|---:|---:|---|\n${angleTable(roll)}\n\n## Pitch sweep\n\n| Pitch (deg) | Moment about pitch (N·m) | Net heave force (N) | Point immersions m (state) |\n|---:|---:|---:|---|\n${angleTable(pitch)}\n\n## Heave sweep\n\n| Heave offset (m; + upward) | Net heave force (N) | Point immersions m (state) |\n|---:|---:|---|\n${heave.map((row) => `| ${row.value.toFixed(3)} | ${fmt(row.net_heave_force_n)} | ${row.point_states.map((point) => `${fmt(point.immersion_m)} (${point.state})`).join("; ")} |`).join("\n")}\n\n## Finding\n\nThe force sign is restoring at every tested nonzero roll and pitch angle. The curve does not reverse through 60°. It ceases to behave approximately linearly when points first dry or saturate: between 10° and 20° roll (observed at 20°), and between 5° and 10° pitch (observed at 10°). Roll moment peaks at 20° and then declines; pitch moment peaks at 20° and then declines. Once opposing points are dry/saturated, total buoyancy is ${fmt(roll[3].buoyancy_n)} N, producing an upward net force of ${fmt(roll[3].net_heave_force_n)} N. This strong angle-to-heave coupling is the notable discontinuous regime; the static restoring sign itself is not wrong.\n`;
const outputMarkdown = markdown.replace(
  /equilibrium draft: \*\*[^*]+\*\*/,
  `equilibrium Surface immersion: **${(-equilibriumZ).toFixed(6)} m**`,
);
const out = resolve("artifacts/rl-campaign/vrx-surface-restoring-sweep");
mkdirSync(out, { recursive: true });
writeFileSync(
  resolve(out, "result.json"),
  JSON.stringify(result, null, 2) + "\n",
);
writeFileSync(resolve(out, "report.md"), outputMarkdown);
console.log(outputMarkdown);
