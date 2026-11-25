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
  resolves false with the arm holding at full gains, as the Python `stop()` leaves it before the
  interrupt propagates.
- `attach()` gravity: the Python SDK eases the new session's gravity feed-forward in from zero over
  `RAMP_S`, so a loaded joint the previous owner was holding dips for half a second at takeover
  (about 3.5 deg on a Metal elbow). With `ArmOptions.attachGravity: "full"` (the default) the
  feed-forward starts at full level when every joint of the gravity chain reports torque on; it is
  still eased in when any of them reports torque off or no torque state. `"ramp"` is the Python
  behaviour (the frame replay uses it). The Python SDK should get the same change.
- Frame spacing: Python busy-waits `FRAME_SPACING_S` start to start; TypeScript sleeps the same gap
  on its clock (a RealClock spins the last stretch of a short sleep).

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
