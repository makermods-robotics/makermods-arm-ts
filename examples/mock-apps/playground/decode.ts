/**
 * A reply's end-effector head to joint rows, as the playground does it (makermods_playground.dream.decode): per arm
 * the tool poses (`action.<side>_eef_xyz`, `_eef_rot6`, the rotation's first two columns) and grip openings, one row
 * per 1/actionHz, smoothed (Savitzky-Golay) and solved by the arm's trajectory IK from the commands already sent.
 */

import { type IKStream, smoothPoses } from "@makermods/metal-arm";

export const SMOOTH_WINDOW = 15;
export const SMOOTH_POLYORDER = 3;
export const MAX_ROWS = 256;

export interface Head {
  xyz: number[][];
  rot6: number[][];
  grip: number[];
}

const finiteRows = (rows: unknown, width: number): rows is number[][] =>
  Array.isArray(rows) && rows.every((r) => Array.isArray(r) && r.length === width && r.every((v) => Number.isFinite(v)));

/** One arm's head from a reply's per_key, or an Error naming what is wrong. */
export function headFromReply(perKey: Record<string, unknown> | undefined, side: "left" | "right"): Head {
  const xyz = perKey?.[`action.${side}_eef_xyz`];
  const rot6 = perKey?.[`action.${side}_eef_rot6`];
  const rawGrip = perKey?.[`action.${side}_grip`];
  const grip = Array.isArray(rawGrip) ? rawGrip.map((g) => (Array.isArray(g) ? g[0] : g)) : null;
  if (!finiteRows(xyz, 3) || !finiteRows(rot6, 6) || !grip || grip.length !== xyz.length || !grip.every(Number.isFinite)
      || xyz.length < 2 || xyz.length > MAX_ROWS || rot6.length !== xyz.length) {
    throw new Error(`${side}: the reply has no finite end-effector head (action.${side}_eef_xyz, _eef_rot6, _grip)`);
  }
  return { xyz, rot6, grip: grip as number[] };
}

const norm = (v: number[]) => Math.hypot(...v);
const cross = (a: number[], b: number[]) => [a[1]! * b[2]! - a[2]! * b[1]!, a[2]! * b[0]! - a[0]! * b[2]!, a[0]! * b[1]! - a[1]! * b[0]!];

/** (T, 4, 4) tool poses: Gram-Schmidt on the two columns. */
export function headPoses(head: Head): number[][][] {
  return head.xyz.map((p, i) => {
    const r = head.rot6[i]!;
    const a1 = r.slice(0, 3);
    const a2 = r.slice(3, 6);
    const b1 = a1.map((v) => v / norm(a1));
    const d = b1[0]! * a2[0]! + b1[1]! * a2[1]! + b1[2]! * a2[2]!;
    const u = a2.map((v, k) => v - d * b1[k]!);
    const b2 = u.map((v) => v / norm(u));
    const b3 = cross(b1, b2);
    return [[b1[0]!, b2[0]!, b3[0]!, p[0]!], [b1[1]!, b2[1]!, b3[1]!, p[1]!], [b1[2]!, b2[2]!, b3[2]!, p[2]!], [0, 0, 0, 1]];
  });
}

/** The 4x4 pose's rotation as rot6 (its first two columns). */
export const poseToRot6 = (T: readonly (readonly number[])[]) => [T[0]![0]!, T[1]![0]!, T[2]![0]!, T[0]![1]!, T[1]![1]!, T[2]![1]!];

/**
 * Joint rows for `head`, continuing the commands `stream` has sent; with the IK's worst position miss, m. The solve
 * yields to the event loop as it goes (feedAsync): solved in one piece it holds the page for longer than the arm's
 * feedback timeout, and the arm's control loop, on the same thread, would fault.
 */
export async function solveHead(stream: IKStream, head: Head): Promise<{ q: number[][]; grip: number[]; posErr: number }> {
  const result = await stream.feedAsync(smoothPoses(headPoses(head), SMOOTH_WINDOW, SMOOTH_POLYORDER));
  return { q: result.q, grip: head.grip.map((g) => Math.min(1, Math.max(0, g))), posErr: Math.max(...result.posErr) };
}
