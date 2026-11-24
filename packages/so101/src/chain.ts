/** The SO-101's kinematic chain, as data for forward kinematics and IK (eePose / moveEe are not implemented in this
package yet). From the console's planner/kinematics.ts and planner/ik.ts: dream_ik/assets/so101_faas_<side>.urdf
(dreamlabs 0.0.30; TheRobotStudio's so101_new_calib.urdf on each side's served_base, the two sides differing in the
base offset and the reference arm's calibrated limits) and specs.so101_faas.

Evaluated as URDF does: per joint, its origin (xyz, then fixed-axis roll-pitch-yaw, R = Rz(yaw)·Ry(pitch)·Rx(roll))
in the parent link, then a turn of the joint angle about its axis; a fixed joint has no axis. joint6 is the wire's
constant-zero pad, a branch off finger_lf that moves nothing on the tool path. The tool is so101_faas's TCP:
tool = flange · [toolRotation | toolOffset], toolRotation's rows the tool's axes in the flange frame (x = -finger_lf
x, y = finger_lf y, z = -finger_lf z, the approach). Joints are radians from the LeRobot calibration's middle pose,
`(ticks - 2048) * 2π / 4096`, with the calibration aligned to Dream's reference arm: So101Arm's joint positions. */

/** One URDF joint as written: origin xyz (m) and rpy (rad) in the parent link; `axis` null for a fixed joint. */
export interface UrdfJoint {
  name: string;
  xyz: readonly [number, number, number];
  rpy: readonly [number, number, number];
  axis: readonly [number, number, number] | null;
  /** [lower, upper] rad, revolute joints only. */
  limit?: readonly [number, number];
  /** A joint hanging off the chain (joint6, the pad): it takes its angle, but moves nothing on the tool path. */
  branch?: boolean;
}

/** One side's chain root to flange and dream_ik's tool on the flange. */
export interface So101Chain {
  side: "left" | "right";
  chain: readonly UrdfJoint[];
  toolOffset: readonly [number, number, number];
  /** Row-major 3×3: the tool's axes in the flange frame. */
  toolRotation: readonly number[];
  /** specs.so101_faas: the IK's rest pose (rad, joint1..joint6). */
  ikRest: readonly number[];
  /** specs.so101_faas: the IK's rest weights. */
  ikRestWeight: readonly number[];
  /** specs.so101_faas: every joint's speed cap (rad/s). */
  ikMaxSpeed: readonly number[];
}

function so101Chain(side: "left" | "right", limits: ReadonlyArray<readonly [number, number]>, ikRest: number[]): So101Chain {
  return {
    side,
    chain: [
      { name: "arm_root_joint", xyz: [0.1775, side === "left" ? -0.125 : 0.125, -0.003], rpy: [0, 0, 0], axis: null },
      { name: "joint1", xyz: [0.0388353, -8.97657e-9, 0.0624], rpy: [3.14159, 4.18253e-17, -3.14159], axis: [0, 0, 1], limit: limits[0] },
      { name: "joint2", xyz: [-0.0303992, -0.0182778, -0.0542], rpy: [-1.5708, -1.5708, 0], axis: [0, 0, 1], limit: limits[1] },
      { name: "joint3", xyz: [-0.11257, -0.028, 1.73763e-16], rpy: [-3.63608e-16, 8.74301e-16, 1.5708], axis: [0, 0, 1], limit: limits[2] },
      { name: "joint4", xyz: [-0.1349, 0.0052, 3.62355e-17], rpy: [4.02456e-15, 8.67362e-16, -1.5708], axis: [0, 0, 1], limit: limits[3] },
      { name: "joint5", xyz: [5.55112e-17, -0.0611, 0.0181], rpy: [1.5708, 0.0486795, 3.14159], axis: [0, 0, 1], limit: limits[4] },
      { name: "joint6", xyz: [0, 0, 0], rpy: [0, 0, 0], axis: [0, 0, 1], limit: [-0.001, 0.001], branch: true },
    ],
    toolOffset: [-0.0097, -0.0002, -0.09653],
    toolRotation: [-1, 0, 0, 0, 1, 0, 0, 0, -1],
    ikRest,
    ikRestWeight: [0.15, 0.3, 0.3, 0.3, 0.3, 0.3],
    ikMaxSpeed: Array(6).fill(5.0),
  };
}

/** Both sides' chains (dream_ik/assets/so101_faas_{left,right}.urdf, verbatim). */
export const SO101_CHAINS: Readonly<Record<"left" | "right", So101Chain>> = {
  left: so101Chain(
    "left",
    [[-1.46802, 2.06934], [-1.788622, 1.842311], [-1.785554, 1.553923], [-1.782486, 1.839243], [-3.092913, 3.188738]],
    [0.3409, -1.7104, 1.5285, 1.1724, -1.4119, 0.0],
  ),
  right: so101Chain(
    "right",
    [[-1.958893, 1.366777], [-1.725728, 1.823903], [-1.917476, 1.486427], [-1.984971, 1.843845], [-3.092913, 3.188738]],
    [-0.3722, -1.6763, 1.459, 1.0564, -1.4099, 0.0],
  ),
};
