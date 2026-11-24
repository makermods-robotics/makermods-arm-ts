# @makermods/so101

TypeScript SDK for the SO-101 arm (six Feetech STS3215 servos on a CH343 USB serial adapter): run
it from a web page over Web Serial, with the same calls as the other MakerMods arms.

```ts
import { createArm, LocalStorageCalibrationStore, requestPort } from "@makermods/so101";

const port = await requestPort();                // call from a click
const arm = createArm(port!, { calibrationStore: new LocalStorageCalibrationStore() });
await arm.connect();
await arm.enable();                              // refused until the arm is calibrated
await arm.moveTo(target);                        // radians
await arm.stop();                                // returns to rest, then releases torque
```

- Calibration by hand or automatic, Wiggle and Swing-to-find are included.
- `@makermods/so101/testing`: a simulated servo bus for application tests.
