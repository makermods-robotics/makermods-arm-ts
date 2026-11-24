/**
 * The arm in a Web Worker (examples/mock-apps/arm-worker): the page picks the adapter from the chooser, the worker
 * finds it with grantedAdapters() and runs the arm and its control loop; the page only shows status.
 */

import type { Page } from "@playwright/test";
import { expect, hook, maxAbsDiff, openApp, tablePositions, test } from "./fixtures.ts";

for (const product of ["metal", "maker"] as const) {
  test.describe(`${product} arm in a worker`, () => {
    async function connect(page: Page): Promise<void> {
      await openApp(page, `/arm-worker/?mock=${product}`);
      await page.selectOption("#product", product);
      await page.click("#connect");
      await expect(page.locator("#message")).toContainText("in the worker: 7 joints");
      await expect(page.locator("#state")).toHaveText("connected");
    }
    const positions = (page: Page) => hook<number[]>(page, "positions");
    const enabled = (page: Page) => hook<boolean[]>(page, "enabled");

    test("connect, enable, move and rest from the worker; the page stays responsive", async ({ page }) => {
      await connect(page);
      await expect(page.locator("#joints tr")).toHaveCount(7);
      // The motors the worker opened answer the hooks (the page's own copy of the adapter is never opened).
      await expect.poll(async () => maxAbsDiff(Object.values(await tablePositions(page)), await positions(page))).toBeLessThan(1e-3);
      const rest = await positions(page);

      await page.click("#enable");
      await expect(page.locator("#message")).toHaveText("Holding the present pose.");
      await expect(page.locator("#state")).toHaveText("enabled");
      expect(await enabled(page)).toEqual(new Array(7).fill(true));

      const stallBefore = Number(await page.locator("#stall").textContent());
      await page.selectOption("#joint", "wrist_flex");
      await page.fill("#delta", "0.3");
      await page.click("#move");
      // While the worker moves the arm, the page answers clicks at once.
      for (let i = 1; i <= 5; i++) {
        const t0 = Date.now();
        await page.click("#ping");
        await expect(page.locator("#pings")).toHaveText(String(i));
        expect(Date.now() - t0).toBeLessThan(1000);
      }
      await expect(page.locator("#message")).toContainText("Arrived: wrist_flex", { timeout: 15_000 });
      const target = JSON.parse((await page.locator("#target").textContent())!) as number[];
      // moveTo reports arrival within its tolerance; the hold then settles on the target.
      await expect.poll(async () => maxAbsDiff(await positions(page), target)).toBeLessThan(0.005);
      // Status kept arriving, and the page's main thread never stalled for long.
      const u0 = Number(await page.locator("#updates").textContent());
      await page.waitForTimeout(2000);
      const perSecond = (Number(await page.locator("#updates").textContent()) - u0) / 2;
      expect(perSecond, "status messages per second (the worker posts every 100 ms)").toBeGreaterThan(7);
      const stall = Number(await page.locator("#stall").textContent());
      expect(stall, `longest main-thread stall ${stall} ms (before the move ${stallBefore} ms)`).toBeLessThan(250);

      await page.click("#rest");
      await expect(page.locator("#message")).toHaveText("At rest, torque released.", { timeout: 20_000 });
      expect(maxAbsDiff(await positions(page), rest)).toBeLessThan(0.02);
      expect(await enabled(page)).toEqual(new Array(7).fill(false));
      await expect(page.locator("#state")).toHaveText("connected");
    });

    test("E-stop in the worker turns every motor off", async ({ page }) => {
      await connect(page);
      await page.click("#enable");
      await expect(page.locator("#state")).toHaveText("enabled");
      await page.click("#estop");
      await expect(page.locator("#message")).toHaveText("E-stop: torque released on every motor.");
      expect(await enabled(page)).toEqual(new Array(7).fill(false));
      await page.click("#disconnect");
      await expect(page.locator("#message")).toHaveText("Disconnected.");
      await expect(page.locator("#state")).toHaveText("not connected");
      expect(await hook(page, "protocolErrors")).toEqual([]);
    });
  });
}
