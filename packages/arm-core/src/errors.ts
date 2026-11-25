/** Error types. Setup and command calls throw them; the control loop reports through the arm's fault state. */

export class ArmError extends Error {
  constructor(message?: string, options?: ErrorOptions) {
    super(message, options);
    this.name = new.target.name;
  }
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
