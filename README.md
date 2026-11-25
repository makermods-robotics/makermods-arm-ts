# MakerMods arm SDKs for TypeScript

Drive MakerMods arms from a web page or from Node. The packages follow the Python SDKs
([maker-arm-sdk](https://github.com/makermods-robotics/maker-arm-sdk),
[metal-arm-sdk](https://github.com/makermods-robotics/metal-arm-sdk)): the same classes and method
names in camelCase, the same safety behaviour, and the same frames on the CAN bus.

| Package | Arm | Link |
|---|---|---|
| `@makermods/metal-arm` | Metal Arm, Damiao motors | WebUSB (candleLight / gs_usb adapter) |
| `@makermods/maker-arm` | Maker Arm, RobStride motors | WebUSB (candleLight / gs_usb adapter) |
| `@makermods/so101` | SO-101, Feetech servos | Web Serial |
| `@makermods/arm-core` | shared by the arm packages; installed with them | |

WebUSB and Web Serial need a Chromium browser and a secure page (https or localhost).

`docs/hardware-testing.md` lists what has run on real arms and what has not.

## In the browser

```ts
import { createArm, GsUsbBackend, requestAdapter } from "@makermods/metal-arm";

button.onclick = async () => {
  const device = await requestAdapter();      // the browser's device chooser; needs a click
  if (!device) return;
  const arm = createArm(new GsUsbBackend(device));
  await arm.connect();                         // reads every motor, never enables torque
  await arm.enable();                          // holds the present pose
  const q = arm.getJointPositions();           // radians
  q[5] += 0.3;
  await arm.moveTo(q, { speed: 0.5 });
  await arm.stop();                            // returns to rest, then releases torque
  await arm.disconnect();
};
```

`arm.setJointTargets(q)` stores a target; the arm's own loop limits and sends it. `arm.estop()`
releases torque at once. `arm.stop()` releases torque only after a verified return to rest and a
fresh recheck of the pose; otherwise, or when its `signal` is aborted (`stop({ signal })`), it
resolves false with the arm holding at full gains.

The control loop runs on the thread the arm was created on. Keep long synchronous work off that
thread (use the `...Async` IK solves, or run the arm in a Web Worker): while it runs no frames go
out and the motors hold their last command; the next tick moves the commands by the time that
passed, at most four periods. `arm.loopRateHz` (also in `status()`) shows the rate the loop keeps,
and a warning is logged once per session when it stays below 90 % of the profile's rate.

## In Node

```ts
import { createArm } from "@makermods/maker-arm";
import { openAdapter } from "@makermods/maker-arm/node";   // needs the optional `usb` package

const arm = createArm(await openAdapter("gs_usb:<serial>"));
```

`examples/node/` has a read-only scan and the sine tracking check (the Python
SDK's `check --sine`). They need a rig file saying which arm is on which adapter,
`~/.config/makermods-arm/rig.json`: `{"gs_usb:<serial>": "metal"}`. `examples/browser/` is a page
that connects, holds, waves the wrist and returns to rest (`npx vite examples/browser`).
`examples/mock-apps/` holds small applications that run on fake adapters and simulated motors.

## Two arms, leaders, cameras

```ts
import { MAKER_ARM, Rig, Teleop, StarLeader, requestSerialPort } from "@makermods/maker-arm";

// The same keys as the Python SDK's rig file. The adapters must already be granted to the page.
const rig = Rig.fromRaw({ arms: { left: { backend: "gs_usb", port: "gs_usb:<serial>" },
                                  right: { backend: "gs_usb", port: "gs_usb:<serial>" } },
                          cameras: { top: "label:Top Cam" } }, MAKER_ARM);
const arms = await (await rig.group()).open();   // connects both
await arms.enable();
await arms.get("left").moveTo(q);
await arms.close();                               // rest, release, disconnect

const leader = new StarLeader((await requestSerialPort())!, { product: MAKER_ARM });
await new Teleop(leader, arm).run(abort.signal);  // the follower tracks the leader until aborted
```

Also in every arm package: `ArmLeader` (a second arm moved by hand as leader), `SlcanSerialBackend`
(USB-serial CAN adapters over Web Serial), `Camera` (`getUserMedia`, or a `url:` camera another
program serves as MJPEG over HTTP, with frame ages and `frame.toJpeg()`).

## Without hardware

```ts
import { simulatedArm } from "@makermods/maker-arm/testing";

const [arm, sim] = simulatedArm();
await arm.connect();
await arm.enable();
sim.fault("elbow_flex");
```

## Development

```
npm install
npm test            # vitest, simulated motors only
npm run typecheck
npm run build
npm run test:browser   # the mock applications in headless Chromium (Playwright)
npm run test:dist      # the built packages, bundled and run in Chromium
```


`docs/porting.md` has the rules that keep the packages in step with the Python SDKs. `tools/` holds
the generators that run the Python SDKs and write `contract/`: the shipped profiles and URDFs, the
public API manifests, and the frame and math fixtures the tests replay.

Not ported from the Python SDKs: the command-line tool, SocketCAN (a Linux kernel interface) and the
JAX trajectory solver.
