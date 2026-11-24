/** The leader interface the teleop loop drives. */

import { ArmError } from "../errors.js";

/** A leader could not be connected or configured, or was lost. */
export class LeaderError extends ArmError {}

/**
 * A source of follower joint targets.
 *
 * `read()` resolves to one target per follower joint (rad, follower joint coordinates, already
 * inside the follower's limits), or null when this cycle has no trustworthy reading. It must not
 * take much longer than one control period; the caller counts consecutive nulls and stops
 * following after too many.
 */
export interface Leader {
  readonly name: string;
  /** Make the leader readable. `signal` aborts a connect that energizes anything. */
  connect(signal?: AbortSignal): Promise<void>;
  /** `signal` (the loop's stop) may cut a slow read short; the caller discards what comes after it. */
  read(signal?: AbortSignal): Promise<number[] | null>;
  close(): Promise<void>;
}

/** Does `value` have the Leader shape (a name and connect, read and close methods)? */
export function isLeader(value: unknown): value is Leader {
  const v = value as Partial<Record<keyof Leader, unknown>> | null;
  return typeof v === "object" && v !== null && typeof v.name === "string"
    && typeof v.connect === "function" && typeof v.read === "function" && typeof v.close === "function";
}
