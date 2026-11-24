/** Error types. Setup and command calls throw them; the control loop reports through the arm's fault state. */

export class ArmError extends Error {
  constructor(message?: string, options?: ErrorOptions) {
    super(message, options);
    this.name = new.target.name;
  }
}

export class ConnectError extends ArmError {}

export class ParamTimeout extends ArmError {}

export class StateError extends ArmError {}

/** A command that would clear fault evidence was refused because a fault is latched. */
export class FaultLatched extends ArmError {}

/** A value failed validation (a config field, an argument). */
export class ValueError extends ArmError {}

/** A bus write did not complete in time. */
export class TimeoutError extends ArmError {}
