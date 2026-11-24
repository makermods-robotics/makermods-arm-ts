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
