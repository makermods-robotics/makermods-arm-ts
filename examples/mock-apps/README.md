# Mock apps

Small applications built on the arm SDKs the way an application would use them, each runnable on simulated hardware
in a real browser. `e2e/` drives them with Playwright in headless Chromium, through their buttons, with no hardware
and no network.

```
npx vite --config examples/mock-apps/vite.config.ts      # then open the page it prints
npx playwright test                                       # the browser tests (they start Vite themselves)
```

The apps run against the package sources (Vite aliases `@makermods/*` to `packages/*/src`, and
`@makermods/dreamlabs` to `../dreamlabs-ts/src` when that checkout exists), so nothing is built first.

| App | What it exercises |
|---|---|
| `arm-console/` | Metal or Maker Arm: `requestAdapter()` on Connect, `createArm(new GsUsbBackend(device))`, live status, Enable, Move, a sine wave, Rest and release, E-stop, unplug handling; or on `SlcanSerialBackend` after `requestSlcanPort()` |
| `arm-worker/` | The same arm and its control loop in a Web Worker: the page requests the adapter, the worker opens it from `grantedAdapters()` and posts status back |
| `teleop/` | A Star Arm 102 leader (`simulatedStarLeader`, its servos on sliders) driving a follower through `Teleop`: slow sync, speed cap, stop and resume, a lost leader, a follower fault |
| `arm-follow/` | `ArmLeader`: a floated leader arm on one adapter, a follower on another |
| `rig/` | Two adapters granted by two chooser clicks, a `Rig` naming them `left` and `right`, an `ArmGroup`: connect, enable, move each, bimanual sine, stop, e-stop; bimanual teleop with two Star leaders through `runMany` |
| `cameras/` | `listCameras`, `Camera` (open, `read`, `newest`, frame age), `Frame.toJpeg()` |
| `timing/` | The control loop's tick rate at each arm's shipped rate, the real Arm on GsUsbBackend over a fake adapter, one arm and two arms at once, in the page and in a worker |
| `so101/` | SO-101 over Web Serial: `requestPort()`, calibration by hand, enable (refused until calibrated), move, wiggle, rest |
| `playground/` | Camera frames (the SDK's `Camera`, `Frame.toJpeg()`) to `DreamClient({ proxyUrl })`, the end-effector chunks solved with the arm's `IKStream` (`feedAsync`) and played to a Metal Arm |

## Simulated hardware

`?mock=metal`, `?mock=maker` or `?mock=so101` in an app's URL loads `mock/install.ts` before the app (through
`shared/mock-gate.ts`), which replaces `navigator.usb` (a `FakeGsUsbDevice` with that arm's simulated motors behind
it, the chooser included; `&adapters=2` plugs in two; module workers the page starts get their own copy through
`mock/worker-entry.ts`) or `navigator.serial` (a `SimSerialPort` on a simulated SO-101, or with `&link=slcan` a
`FakeSerialPort` SLCAN adapter with the arm's motors behind it). The Star leaders are the SDK's
`simulatedStarLeader`, turned by the apps' sliders. `window.__mock` holds the test hooks: the simulated
motors' positions and torque, injected faults, unplugging and replugging the adapter, moving a limp SO-101 by hand.
Without `?mock` the apps talk to real adapters.

The playground's policy side is not in the page: `e2e/dream-server.ts` runs the Dream client's proxy
(`createDreamProxy` + `nodeHandler`, holding a throwaway key) in front of a stand-in gateway that answers chunks
recorded from the Python client (`../dreamlabs-ts/test/fixtures/wire.json`). Its tests are skipped without that
checkout.
