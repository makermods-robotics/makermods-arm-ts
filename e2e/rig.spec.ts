/**
 * The two-arm robot (examples/mock-apps/rig): two fake adapters granted by two chooser clicks, a Rig naming them as
 * left and right, driven as an ArmGroup; and bimanual teleop with two simulated Star leaders through runMany().
 */

import type { Page } from "@playwright/test";
import { expect, hook, maxAbsDiff, openApp, slide, test } from "./fixtures.ts";

for (const product of ["metal", "maker"] as const) {
  test.describe(`two ${product} arms`, () => {
    const serial = (n: number) => `FAKE-${product.toUpperCase()}-${n}`;
    const of = (page: Page, side: "left" | "right", op: string, ...args: unknown[]) =>
      hook<number[] | boolean[]>(page, "callOn", serial(side === "left" ? 1 : 2), op, ...args);
    const positions = (page: Page, side: "left" | "right") => of(page, side, "positions") as Promise<number[]>;
    const enabled = (page: Page, side: "left" | "right") => of(page, side, "enabled") as Promise<boolean[]>;

    async function grantBoth(page: Page): Promise<void> {
      await openApp(page, `/rig/?mock=${product}&adapters=2`);
      await page.selectOption("#product", product);
      await page.click("#grant");
      await page.click("#grant");
      await expect(page.locator("#granted")).toHaveText(`gs_usb:${serial(1)}, gs_usb:${serial(2)}`);
    }

    async function connectBoth(page: Page): Promise<void> {
      await grantBoth(page);
      await page.click("#connect");
      await expect(page.locator("#message")).toHaveText(`Connected: left on gs_usb:${serial(1)}, right on gs_usb:${serial(2)}.`);
      await expect(page.locator("#left-state")).toHaveText("connected");
      await expect(page.locator("#right-state")).toHaveText("connected");
    }

    async function enableBoth(page: Page): Promise<void> {
      await page.click("#enable");
      await expect(page.locator("#message")).toHaveText("Both arms holding.");
      for (const side of ["left", "right"] as const) expect(await enabled(page, side)).toEqual(new Array(7).fill(true));
    }

    test("connect, enable, move each, bimanual sine, stop both", async ({ page }) => {
      test.setTimeout(90_000);
      await connectBoth(page);
      await expect(page.locator("#left-joints tr")).toHaveCount(7);
      await expect(page.locator("#right-joints tr")).toHaveCount(7);
      const rest = { left: await positions(page, "left"), right: await positions(page, "right") };
      await enableBoth(page);

      await page.selectOption("#joint", "wrist_flex");
      await page.fill("#delta", "0.3");
      await page.click("#move-left");
      await expect(page.locator("#message")).toContainText("left arrived: wrist_flex", { timeout: 15_000 });
      await expect.poll(async () => (await positions(page, "left"))[3]! - rest.left[3]!).toBeGreaterThan(0.29);
      expect(maxAbsDiff(await positions(page, "right"), rest.right)).toBeLessThan(1e-3); // the other arm stays put
      await page.fill("#delta", "-0.2");
      await page.click("#move-right");
      await expect(page.locator("#message")).toContainText("right arrived: wrist_flex", { timeout: 15_000 });
      await expect.poll(async () => (await positions(page, "right"))[3]! - rest.right[3]!).toBeLessThan(-0.19);

      await page.click("#sine");
      await expect(page.locator("#message")).toContainText("Sine done", { timeout: 20_000 });
      const amplitudes = JSON.parse((await page.locator("#sine").getAttribute("data-amplitudes"))!) as Record<string, number>;
      for (const side of ["left", "right"]) expect(Math.abs(amplitudes[side]! - (10 * Math.PI) / 180), side).toBeLessThan(0.02);

      await page.click("#stop");
      await expect(page.locator("#message")).toHaveText("Stopped: left at rest, released; right at rest, released.", { timeout: 30_000 });
      for (const side of ["left", "right"] as const) {
        expect(maxAbsDiff(await positions(page, side), rest[side]), side).toBeLessThan(0.02);
        expect(await enabled(page, side)).toEqual(new Array(7).fill(false));
      }
    });

    test("E-stop turns both arms off", async ({ page }) => {
      await connectBoth(page);
      await enableBoth(page);
      await page.click("#estop");
      await expect(page.locator("#message")).toHaveText("E-stop: torque released on both arms.");
      for (const side of ["left", "right"] as const) expect(await enabled(page, side)).toEqual(new Array(7).fill(false));
    });

    test("one arm faulting leaves the other holding, each reported", async ({ page }) => {
      await connectBoth(page);
      await enableBoth(page);
      const left = await positions(page, "left");
      await of(page, "right", "fault", "elbow_flex");
      await expect(page.locator("#right-state")).toHaveText("fault");
      await expect(page.locator("#right-fault")).toContainText("elbow_flex");
      await expect(page.locator("#left-state")).toHaveText("enabled");
      await expect(page.locator("#left-fault")).toHaveText("");
      await page.waitForTimeout(500);
      expect(await enabled(page, "left")).toEqual(new Array(7).fill(true));
      expect(maxAbsDiff(await positions(page, "left"), left)).toBeLessThan(1e-3);
      await expect(page.locator("#move-left")).toBeEnabled();
      await expect(page.locator("#move-right")).toBeDisabled();
    });

    test("a rig naming an adapter that was not granted is refused before any frame is sent", async ({ page }) => {
      await grantBoth(page);
      const rig = JSON.parse(await page.locator("#rig-text").inputValue());
      rig.arms.right.port = `gs_usb:${serial(9)}`;
      await page.fill("#rig-text", JSON.stringify(rig));
      await page.click("#connect");
      await expect(page.locator("#message")).toContainText(`no gs_usb adapter with serial '${serial(9)}'`);
      await expect(page.locator("#connect")).toBeEnabled();
      expect(await hook(page, "framesBySerial")).toEqual({ [serial(1)]: 0, [serial(2)]: 0 });
      expect(await hook(page, "opensBySerial")).toEqual({ [serial(1)]: 0, [serial(2)]: 0 });
    });

    test("bimanual teleop: two Star leaders through runMany", async ({ page }) => {
      await connectBoth(page);
      await enableBoth(page);
      await page.click("#teleop-start");
      await expect(page.locator("#teleop")).toHaveText("running");
      await page.waitForTimeout(300);
      const before = { left: await positions(page, "left"), right: await positions(page, "right") };
      await slide(page, "#left-star-servo-3", 25);
      await slide(page, "#right-star-servo-5", -40);
      await expect.poll(async () => Math.abs((await positions(page, "left"))[3]! - before.left[3]!), { timeout: 10_000 }).toBeGreaterThan(0.3);
      await expect.poll(async () => Math.abs((await positions(page, "right"))[5]! - before.right[5]!), { timeout: 10_000 }).toBeGreaterThan(0.5);
      expect(Math.abs((await positions(page, "left"))[5]! - before.left[5]!)).toBeLessThan(0.02); // each leader its own arm
      // One leader lost stops both pairs; both arms hold.
      await page.locator("#right-star-online-1").uncheck();
      await expect(page.locator("#teleop")).toContainText("stopped", { timeout: 10_000 });
      await expect(page.locator("#teleop")).toContainText("right: leader_lost");
      await expect(page.locator("#teleop")).toContainText("left: stopped");
      for (const side of ["left", "right"] as const) expect(await enabled(page, side)).toEqual(new Array(7).fill(true));
    });
  });
}
