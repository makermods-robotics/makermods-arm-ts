/**
 * Transport-layer interface: moves CAN frames only, no protocol knowledge.
 *
 * The receive callback must be fast in / fast out (parse and cache); it must not await. There is
 * one callback per backend; a component that needs to observe every frame for a while reads
 * `recvCallback`, installs a callback that chains to it, and puts it back afterwards.
 */

/** `extended` is true for a 29-bit identifier. */
export type RecvCallback = (canId: number, data: Uint8Array, extended: boolean) => void;

export abstract class CanBackend {
  /**
   * Adapters that cannot absorb a burst of replies set this: the control loop then waits for each
   * motor's reply (up to feedbackWaitTimeout seconds) before commanding the next one.
   */
  feedbackPacedControl = false;
  feedbackWaitTimeout = 0.015;

  protected cb: RecvCallback | null = null;

  abstract open(): Promise<void>;

  abstract close(): Promise<void>;

  /**
   * Queue one classic CAN frame (at most 8 data bytes). Frames go out in call order. Resolves once
   * the adapter has taken the frame; rejects when it could not be written.
   */
  abstract send(canId: number, data: Uint8Array, extended?: boolean): Promise<void>;

  /** Every received frame goes to `cb(canId, data, extended)` (null: frames are dropped). */
  setRecvCallback(cb: RecvCallback | null): void {
    this.cb = cb;
  }

  /** The callback installed by setRecvCallback(), or null. */
  get recvCallback(): RecvCallback | null {
    return this.cb;
  }
}
