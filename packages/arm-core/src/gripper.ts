/**
 * Gripper holding-effort regulation.
 *
 * Free closing aims at the requested goal with Kp scaled so Kp x error never exceeds the closing
 * cap (max(1, 2 x hold) N·m) and Kd lowered to CAPPED_CLOSING_KD. Once contact is detected
 * (closing effort > 0.5 N·m at |v| < 0.3 rad/s for 100 ms) the command is regulated so the
 * measured effort settles at the holding target. Opening, or a sustained loss of contact, ends
 * holding.
 */

import { radians } from "./config.js";
import { ValueError } from "./errors.js";

export const CAPPED_CLOSING_KD = 0.02;

/**
 * Python 3.12's sum() of floats (Neumaier-compensated), so averages match the Python SDK bit for
 * bit.
 */
export function pySum(values: Iterable<number>): number {
  let first = true;
  let f = 0;
  let c = 0;
  for (const x of values) {
    if (first) {
      f = 0 + x;
      first = false;
      continue;
    }
    const t = f + x;
    if (Math.abs(f) >= Math.abs(x)) c += (f - t) + x;
    else c += (x - t) + f;
    f = t;
  }
  if (c && Number.isFinite(c)) f += c;
  return f;
}

/**
 * Radians throughout. `update` returns the position to send; read `kpScale` and `closingCapped`
 * afterwards for the gains.
 */
export class GripperHold {
  readonly torqueNm: number;
  readonly limits: readonly [number, number];
  readonly closingDirection: number;
  /** One step of the command frame's 12-bit Kp field over the product's Kp range. */
  readonly kpStep: number;

  holding = false;
  closingCapped = false;
  kpScale = 1.0;
  lastCommand: number | null = null;
  filtered: number | null = null;
  history: Array<[number, number]> = [];
  lastTime: number | null = null;
  contactStarted: number | null = null;
  lowEffortSince: number | null = null;

  /** `kpRange` is the product's protocol Kp range (Product.kpRange). */
  constructor(torqueNm: number, limits: readonly [number, number], kpRange: readonly [number, number],
              closingDirection = 1) {
    if (!(Number.isFinite(torqueNm) && torqueNm >= 0.1 && torqueNm <= 2.0)) {
      throw new ValueError("gripper hold torque must be in [0.1, 2.0] Nm");
    }
    if (closingDirection !== -1 && closingDirection !== 1) throw new ValueError("closing direction must be -1 or +1");
    this.torqueNm = torqueNm;
    this.limits = limits;
    this.closingDirection = closingDirection;
    this.kpStep = (kpRange[1] - kpRange[0]) / 4095;
    this.reset();
  }

  reset(): void {
    this.holding = false;
    this.closingCapped = false;
    this.kpScale = 1.0;
    this.lastCommand = null;
    this.filtered = null;
    this.history = [];
    this.lastTime = null;
    this.contactStarted = null;
    this.lowEffortSince = null;
  }

  get closingCapNm(): number {
    return Math.max(1.0, 2.0 * this.torqueNm);
  }

  update(goal: number, position: number, velocity: number, effort: number, kp: number, now: number): number {
    if (![goal, position, velocity, effort, kp, now].every(Number.isFinite) || kp <= 0) {
      throw new ValueError("invalid gripper holding feedback/gain");
    }
    const s = this.closingDirection;
    const [lo, hi] = this.limits;
    goal = Math.min(hi, Math.max(lo, goal));
    const dt = this.lastTime === null ? 0.0 : Math.max(0.0, now - this.lastTime);
    this.lastTime = now;
    this.history.push([now, effort]);
    while (this.history.length && this.history[0]![0] < now - 0.1) this.history.shift();
    const closingEffort = s * pySum(this.history.map(([, e]) => e)) / this.history.length;
    const opening = s * (goal - position) < -radians(0.25);
    if (this.holding) {
      if (closingEffort < Math.min(0.2, this.torqueNm * 0.4)) {
        if (this.lowEffortSince === null) this.lowEffortSince = now;
      } else {
        this.lowEffortSince = null;
      }
      const lostContact = this.lowEffortSince !== null && now - this.lowEffortSince >= 0.3;
      if (opening || lostContact) {
        this.holding = false;
        this.contactStarted = null;
        this.lowEffortSince = null;
      }
    } else if (s * (goal - position) > 0 && closingEffort > 0.5 && Math.abs(velocity) < 0.3) {
      if (this.contactStarted === null) this.contactStarted = now;
      if (now - this.contactStarted >= 0.1) this.holding = true;
    } else {
      this.contactStarted = null;
    }

    let command: number;
    if (this.holding) {
      const raw = this.lastCommand! + s * (this.torqueNm - Math.abs(effort)) / kp;
      const alpha = 1.0 - Math.exp(-Math.min(dt, 0.05) / 0.19);
      this.filtered = this.filtered! + alpha * (raw - this.filtered!);
      const bounded = s < 0 ? Math.max(goal, this.filtered) : Math.min(goal, this.filtered);
      command = Math.min(hi, Math.max(lo, bounded));
      this.filtered = command;
    } else {
      command = goal;
      this.filtered = position;
    }
    this.closingCapped = false;
    this.kpScale = 1.0;
    let sent = command;
    const lead = this.closingCapNm / kp;
    const error = s * (command - position);
    if (error > lead) {
      const equivalent = Math.min(hi, Math.max(lo, position + s * lead));
      if (this.holding) {
        command = sent = equivalent;
        this.filtered = command;
      } else {
        this.closingCapped = true;
        this.kpScale = lead / error;
        command = equivalent;
      }
    }
    this.lastCommand = command;
    return sent;
  }

  /**
   * Gains for the last update: a capped Kp is rounded DOWN to the 12-bit grid so quantization can
   * never lift Kp x error above the cap.
   */
  gains(kp: number, kd: number): [number, number] {
    if (this.kpScale >= 1.0) return [kp, kd];
    const scaled = kp * this.kpScale;
    return [(Math.floor(scaled / this.kpStep) + 1e-6) * this.kpStep, this.closingCapped ? CAPPED_CLOSING_KD : kd];
  }
}
