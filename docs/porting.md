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
- `attach()` gravity: by default (`ArmOptions.attachGravity: "ramp"`) the feed-forward eases in from
  zero over `RAMP_S`, as the Python SDK does, so a loaded joint the previous owner was holding dips
  for half a second at takeover (about 3.5 deg on a Metal elbow). `"measured"` is an experimental
  opt-in, not yet run on an arm: right before the first holding frame it takes the pose and the
  feedback torques (joint sign) again and starts the ease-in at the level whose capped command,
  clip(model * level, ±gravity_limit_nm) as the sender computes it, best fits them (least squares
  over 0..1 in steps of 0.001); from zero when a chain joint reports torque off or no torque state,
  when a chain joint's reading is older than feedbackTimeout, or when the model torque is under
  `ATTACH_LOAD_MIN_NM`. Both families report an estimated output torque (from motor current) in
  N·m and motor sign, 12 bits over the model's torque range (RobStride tMin..tMax, Damiao ±T_MAX: a
  step of a few thousandths to a few hundredths of a N·m by model), including friction and the
  motor's own error. The Python SDK could get the same option once it has been run on an arm.
- Frame spacing: Python busy-waits `FRAME_SPACING_S` start to start; TypeScript sleeps the same gap
  on its clock (a RealClock spins the last stretch of a short sleep).
- stop(): a weight-carrying joint left out of the return is read again with the returned joints right
  before the release; missing, stale or raised (beyond restSettle from the supported pose) refuses
  it (stricter than Python 28621d6).
- stop(): the release recheck (stricter than Python 28621d6, which reads each motor once and
  counts the first reply after the read). MIT feedback carries no request id and a reading's age is
  its host receipt time, so the reading is established by procedure, and any ambiguity keeps the arm
  holding: after the control loop stops the bus must go quiet (no reply from any motor for
  REPLY_TIMEOUT, within QUIET_BOUND_S); then two read cycles each send the holding frame (no status
  poll) and require exactly one reply from every checked joint after that cycle's request, younger
  than min(feedbackTimeout, REST_FRESH_S); both cycles' poses must meet the release criteria and
  agree within REST_AGREE; no reply may arrive after the second cycle before the release; faults
  and the arm state are judged again right before it. A missing reply, more replies than requests,
  a stale age, a fault, a state change, an abort, or a recheck write that does not complete within
  SEND_TIMEOUT_S refuses the release; nothing "repairs" a missing reply. Guaranteed: every reply
  counted arrived after a quiet window in which nothing was in flight, one per request. Not
  guaranteed: a device that keeps replies longer than the quiet window without delivering any, then
  releases exactly one stored reply per new request, passes if it holds two or more stored replies
  (two stale replies in a row that agree); with one stored reply the second cycle's reading answers
  the first cycle's request, which was sent after the loop stopped.
- stop(): resolves true only when every motor confirmed torque off; otherwise false with the arm
  CONNECTED, the loop stopped, the motors in `unconfirmedDisable` and `status().unconfirmedDisable`,
  and disable() tries again. A later stop() keeps resolving false, and enable() and setMotorId()
  refuse, until disable() has confirmed them; disconnect() reports them and keeps them listed
  (stricter than Python 28621d6, whose stop() reports True both times).
- stop(): every exit that does not release holds the measured pose at the profile's gains, ending
  float mode, setGains() gains and the extra torque first (stricter than Python 28621d6).
- estop(): enable() and attach() called while an e-stop is pending are refused at once (and again
  inside the lifecycle lock, also when an e-stop ran since they were called), so a call issued
  during an e-stop never energizes; estop() does not wait for a lifecycle call in progress (a stop()
  blocked on a write included); stop()'s recheck tick and every command frame after a spacing wait
  check the session, and stop()'s cleanup sends nothing once an e-stop is pending or has run
  (stricter than Python 28621d6). The read-only status query after a spacing wait still goes out, as
  in Python (the frame replay records one after an e-stop).
- Shutdown ownership (stricter than Python 28621d6): detach() during an e-stop runs the confirmed
  release before closing the bus; an e-stop whose release could not run before the bus closed lists
  every motor in unconfirmedDisable; with holdOnFault off the fault path's disables are confirmed
  and the unconfirmed reported; a bus close that fails during detach() restarts the control loop
  (and rejects); writes of stop()'s recheck and of every shutdown are bounded by SEND_TIMEOUT_S (one
  that does not settle counts as failed, or as possibly sent for the point of no return).
- stop(): the point of no return is the first torque-off frame the transport accepted or whose write
  did not settle: an abort before it holds (a write that was rejected did not go out); after it the
  release finishes and its outcome is reported. Every wait of the recheck (the paced reply wait, the
  spacing sleeps, both loop joins) wakes on the abort (stricter than Python 28621d6, where an
  interrupt during the release leaves some motors off and the rest holding).
- When a hold, fault or shutdown is installed during a tick, every frame of that tick not yet sent
  is dropped (position, gains, feed-forward and a regulated gripper alike); the next tick computes
  them afresh (stricter than Python 28621d6, which sends the stale frames). User gains survive
  returnToRest(), a successful stop() and a new enable; only a stop() that does not release restores
  the profile's. The frame replay keeps Python's frames (and Python's single-read recheck) through
  `pythonFrameReplay()` from "/testing", a module-private registry no other code reaches.
- moveEe(): a cancel during the last row's period, or before the arrival wait, holds the measured
  pose (stricter than Python 28621d6, which leaves the last row as the target).
- moveEe() planning: an expired allowance is checked before the path's own replaces it, and the
  straight-line poses are built cooperatively (`straightLineAsync`, a pause every 256 rows); the
  caller stops waiting PLAN_GRACE_S after its timeout or abort even if the planner never settles,
  and gives up the motion so a late result cannot commit (stricter than Python 28621d6).
- setMotorId(): ids must be whole numbers at the arm, motor and protocol boundaries, before the bus
  is opened (stricter than Python 28621d6). The joints at both ids are marked unverified as soon as
  the persistent write may land (Motor.setId's onWrite) and cleared only once the change is
  verified and nothing answers at the old address on any reply id (the arm listens to every frame
  its product routes to the old id); otherwise enable()/attach() refuse that Arm, for both products
  (Python 28621d6 marks only on RouteUnverified and checks the old route only for RobStride, on its
  own reply id).
- The rest press never moves a joint enabled outside its soft limits (inside fullTurnGrace), so
  the clamp to the limits cannot exceed REST_PRESS / REST_PRESS_NM (stricter than Python 28621d6).
- The slow-loop warning goes out once per session, also after a recovery; a loop restarted by
  stop() starts a new loop-rate window (stricter than Python 28621d6).
- url: cameras: MAX_HEADERS counts physical header lines, so a part repeating one header name is bounded too (stricter than Python 28621d6).
- url: cameras: a JPEG without EOI at its end (a truncated part or response) is skipped and counted in `HttpCapture.incomplete`, never published as a frame (stricter than Python 28621d6).

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
