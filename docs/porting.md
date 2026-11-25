# Porting rules

The TypeScript packages follow the Python SDKs (`maker-arm-sdk`, `metal-arm-sdk`): same concepts,
same behaviour, the same frames on the CAN bus. The Python code is the specification.

| Python | TypeScript |
|---|---|
| `maker_arm/` shared files | `packages/arm-core/src/` |
| `maker_arm/` product files (`_product.py`, `protocol.py`, `motor.py`, `_sim_motor.py`, profile, URDF) | `packages/maker-arm/src/` |
| `metal_arm/` product files | `packages/metal-arm/src/` |

## Names

- Files: the Python module in kebab-case (`trajectory_ik.py` → `trajectory-ik.ts`).
- Classes keep their names. Functions, methods and fields become camelCase. Constants stay `UPPER_SNAKE`.
- Private members (`_name`) become `#name` or `private name`.
- Profile keys stay snake_case as in the YAML profiles; `ArmConfig.fromRaw()` converts them.
- Units are SI and radians everywhere, as in Python.
- Keyword arguments become one trailing options object when a function has more than two optional
  parameters (`moveTo(q, { speed, timeout, wait, signal })`); otherwise positional.
- A `cancel: threading.Event` parameter becomes `signal: AbortSignal`.

## Sync and async

- Anything that puts a frame on the bus or waits is `async`. Cached getters and pure functions are sync.
- No threads. A Python thread becomes an async loop. A lock held while waiting on the bus becomes a
  `Mutex`; a lock around plain assignments is dropped.
- A Python polling loop (`while seq == before: sleep(0.0005)`) becomes
  `await signal.wait(() => seq !== before, timeout, clock)`. Loops that sleep a fixed time on
  purpose (`_wait_quiet`) keep their sleeps.
- All time comes from the injected `Clock` (`clock.now()`, `clock.sleep()`, `clock.timer()`), never
  from `Date`, `performance` or `setTimeout` directly, so every test runs on `VirtualClock`.

## Intended differences

- `move_ee` path planning: the Python SDK solves the path in a helper process (`_planner.py`) so the
  solve never holds the interpreter lock its control loop needs. `Arm.moveEe` solves it
  cooperatively on the event loop (`Chain.ikAsync`, `solvePathAsync`, `planner.ts`), pausing every
  few milliseconds of computation. The limits, refusals and return values are the Python SDK's: 5 s
  for the end pose, then 3 s plus 25 ms per path tick, never more than 60 s; a path longer than
  12000 ticks is refused before it is built; `timeout` and `signal` apply while it plans. The budget
  is checked at every pause instead of by a waiting caller. There is no helper to start, kill or
  fall back from (`Arm.PLAN_IN_HELPER` has no counterpart).
- `realtime.py` (pinning the control loop and receive threads to the fast cores of a big.LITTLE
  CPU, and the GIL switch interval) has no JavaScript counterpart: the control loop shares the
  event loop with everything else on its thread.
- A second Ctrl-C during `stop()` (a `KeyboardInterrupt` raised inside it) is `stop({ signal })`:
  aborting the signal ends the return to rest, or the recheck before the release, and `stop()`
  resolves false with the arm holding, as the Python `stop()` leaves it before the interrupt
  propagates. Every wait in stop() (the two-tick wait, the ramp and settle sleeps, the loop join,
  the reply waits) wakes on the abort, as a SIGINT ends a Python sleep; the frame generator models
  the Ctrl-C that way (`stop_interrupted` scenarios).
- `attach()` gravity: the Python SDK eases the new session's gravity feed-forward in from zero over
  `RAMP_S`, so a loaded joint the previous owner was holding dips for half a second at takeover
  (about 3.5 deg on a Metal elbow). With `ArmOptions.attachGravity: "full"` (the default) the ease-in
  starts at the share of the model's holding torque the motors carry, from their fresh feedback
  torques (joint sign): clamp(dot(measured, model) / dot(model, model), 0, 1), and from zero when any
  chain joint reports torque off or no torque state, or the model torque is under
  `ATTACH_LOAD_MIN_NM`. Both families report an estimated output torque (from motor current) in
  N·m and motor sign, 12 bits over the model's torque range (RobStride tMin..tMax, Damiao ±T_MAX: a
  step of a few thousandths to a few hundredths of a N·m by model); the arm applies the joint
  direction. The estimate includes friction and the motor's own error, so the share is approximate. `"ramp"` is the Python behaviour (the frame replay uses it).
  The Python SDK should get the same change.
- Frame spacing: Python busy-waits `FRAME_SPACING_S` start to start; TypeScript sleeps the same gap
  on its clock (a RealClock spins the last stretch of a short sleep).
- stop(): a weight-carrying joint left out of the return is read again with the returned joints right
  before the release; missing, stale or raised (beyond restSettle from the supported pose) refuses
  it (stricter than Python def3243).
- stop(): the reading the recheck counts answers a command sent after the control loop stopped:
  replies are counted against the session's commands (a Damiao refresh is answered with a state
  frame and counted as a command); the loop's replies drain first (up to REPLY_TIMEOUT), and a motor
  still owing one (lost, or held back until the next frame) is read once more before the recheck,
  so a buffered reply up to one frame late cannot pass for the recheck's. A deeper buffer is not
  covered (stricter than Python def3243).
- stop(): motor faults and the arm state are judged again after the recheck's waits, right before
  the release (stricter than Python def3243).
- stop(): the recheck reading must be younger than min(feedbackTimeout, REST_FRESH_S) (stricter than
  Python def3243).
- stop(): resolves true only when every motor confirmed torque off; otherwise false with the arm
  CONNECTED, the loop stopped, the motors in `unconfirmedDisable` and `status().unconfirmedDisable`,
  and disable() tries again (stricter than Python def3243, whose stop() reports True).
- stop(): every exit that does not release holds the measured pose at the profile's gains, ending
  float mode, setGains() gains and the extra torque first (stricter than Python def3243).
- estop(): enable() and attach() refuse while an e-stop's confirmed shutdown is in progress; stop()'s
  recheck tick and every command frame after a spacing wait check the session, and stop()'s cleanup
  sends nothing while an e-stop is pending (stricter than Python def3243). The read-only status
  query after a spacing wait still goes out, as in Python (the frame replay records one after an
  e-stop).
- stop(): the point of no return is the first torque-off frame: an abort before it holds, after it
  the release finishes and its outcome is reported (stricter than Python def3243, where an interrupt
  during the release leaves some motors off and the rest holding).
- A hold, fault or shutdown that supersedes a tick's commands during a spacing wait (or a slow
  send) replaces the frames not yet sent with the held command (stricter than Python def3243, which
  sends the stale frame; the frame replay keeps Python's frames through a private `pythonFrames`
  switch, used by no other code).
- moveEe(): a cancel during the last row's period, or before the arrival wait, holds the measured
  pose (stricter than Python def3243, which leaves the last row as the target).
- moveEe() planning: an expired allowance is checked before the path's own replaces it, and the
  straight-line poses are built cooperatively (`straightLineAsync`, a pause every 256 rows)
  (stricter than Python def3243).
- setMotorId(): ids must be whole numbers at the arm, motor and protocol boundaries, before the bus
  is opened (stricter than Python def3243). After the id change both routes are probed before the
  joint is routed to the new id; something answering at the old id is RouteUnverified and blocks
  enable()/attach() for both products (Python def3243 checks this only for RobStride).
- The rest press never moves a joint enabled outside its soft limits (inside fullTurnGrace), so
  the clamp to the limits cannot exceed REST_PRESS / REST_PRESS_NM (stricter than Python def3243).
- The slow-loop warning goes out once per session, also after a recovery; a loop restarted by
  stop() starts a new loop-rate window (stricter than Python def3243).
- url: cameras: MAX_HEADERS counts physical header lines, so a part repeating one header name is bounded too (stricter than Python def3243).
- url: cameras: a JPEG without EOI at its end (a truncated part or response) is skipped and counted in `HttpCapture.incomplete`, never published as a frame (stricter than Python def3243).

## Bytes and numbers

- Frames are `Uint8Array`. No `Buffer` and no `node:` imports outside files named `node.ts`.
- Arithmetic that feeds a frame keeps Python's operation order, so results match bit for bit.
  `int(x)` truncates toward zero (`Math.trunc`). Python's `round()` rounds half to even; port it as
  such where it is used. `math.radians(x)` is `x * (Math.PI / 180)` (`radians()` in `config.ts`).
- Vectors are `number[]`, matrices and poses `number[][]` (row-major), trajectories `number[][]`.

## Safety

Every check, limit, refusal and fault path in the Python code is ported, with its order of
operations. Nothing is relaxed or "simplified". When JavaScript forces a different mechanism (no
threads), the observable behaviour and the frames stay the same.

## Tests and fixtures

- Tests are vitest files in `packages/<pkg>/test/`. Port the Python unit tests that apply.
- Cross-language fixtures: a Python script in `tools/` runs the Python SDK and writes JSON to
  `contract/fixtures/`; the TypeScript test replays the same inputs and compares. Frame bytes are
  compared exactly; floating-point results from longer computations to 1e-9 or tighter.
- Run generators with the SDK's own interpreter: `~/maker-arm-sdk/.venv/bin/python` or
  `~/metal-arm-sdk/.venv/bin/python`. The Python repositories are never modified.

## Packages

- No runtime dependencies. ESM, imports end in `.js`.
- `arm-core` never names a product and never imports an arm package. Everything product-specific
  reaches it through the `Product` object on `ArmConfig`.
- Public symbols carry a doc comment. Comments describe the current code only.
