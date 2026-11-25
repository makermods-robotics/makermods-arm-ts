/** Error types. Setup and command calls throw them; the control loop reports through the arm's fault state. */

export class ArmError extends Error {
  /**
   * For a failed write: false when it was raised before the frame was handed to the device (it was
   * certainly not sent: the transport was not open, or no transmit slot came free); undefined when
   * that is not known.
   */
  declare submitted?: boolean;

  constructor(message?: string, options?: ErrorOptions) {
    super(message, options);
    this.name = new.target.name;
  }
}

/** Mark `error` as raised before its frame was handed to the device (`submitted` false; not enumerable). */
export function notSubmitted<E extends ArmError>(error: E): E {
  Object.defineProperty(error, "submitted", { value: false, enumerable: false, configurable: true, writable: true });
  return error;
}

export class ConnectError extends ArmError {}

/**
 * enable() refused: the motors are energized, held by another program (take the arm over with
 * attach(), or bring it down with returnToRest() and disable() from that program or the CLI's `rest`).
 */
export class HeldByAnotherProgram extends ConnectError {}

export class ParamTimeout extends ArmError {}

export class StateError extends ArmError {}

/** After an id change it is unclear which motor answers on a joint's id (both ids answer). */
export class RouteUnverified extends StateError {}

/** A command that would clear fault evidence was refused because a fault is latched. */
export class FaultLatched extends ArmError {}

/** A value failed validation (a config field, an argument). */
export class ValueError extends ArmError {}

/** A bus write did not complete in time. */
export class TimeoutError extends ArmError {}
