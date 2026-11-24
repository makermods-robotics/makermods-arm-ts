import { expectTypeOf, it } from "vitest";
import type { UsbDeviceLike } from "@makermods/arm-core/transport/gs-usb";

it("a browser USBDevice is a UsbDeviceLike", () => {
  expectTypeOf<USBDevice>().toExtend<UsbDeviceLike>();
});
