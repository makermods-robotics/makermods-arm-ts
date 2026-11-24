# @makermods/maker-arm

TypeScript SDK for the MakerMods Maker Arm (7 RobStride motors): run it from a web page over WebUSB, or from Node.

```ts
import { createArm, GsUsbBackend, requestAdapter } from "@makermods/maker-arm";

const device = await requestAdapter();          // call from a click
const arm = createArm(new GsUsbBackend(device!));
await arm.connect();
await arm.enable();
await arm.moveTo(target);                        // radians
await arm.stop();                                // returns to rest, then releases torque
```

- `@makermods/maker-arm/testing`: a simulated arm for application tests.
- `@makermods/maker-arm/node`: open an adapter from Node (needs the optional `usb` package).

Same classes and methods as the Python SDK, in camelCase.
