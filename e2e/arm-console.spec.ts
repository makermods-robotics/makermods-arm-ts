/**
 * The arm console (examples/mock-apps/arm-console) on a fake candleLight adapter with each arm's simulated motors:
 * the whole stack runs in the page (chooser, WebUSB transport, GsUsbBackend, Arm), driven through its buttons.
 */

import type { Page } from "@playwright/test";
import { expect, hook, maxAbsDiff, openApp, tablePositions, test } from "./fixtures.ts";


for (const product of ["metal", "maker"] as const) {
  test.describe(`${product} arm console`, () => {
    async function open(page: Page): Promise<void> {
      await openApp(page, `/arm-console/?mock=${product}`);
      await page.selectOption("#product", product);
    }

    async function connect(page: Page): Promise<void> {
      await page.click("#connect");
      await expect(page.locator("#message")).toContainText("Connected to FAKE-");
      await expect(page.locator("#state")).toHaveText("connected");
    }

    async function enable(page: Page): Promise<void> {
      await page.click("#enable");
      await expect(page.locator("#message")).toHaveText("Holding the present pose.");
      await expect(page.locator("#state")).toHaveText("enabled");
    }

    const positions = (page: Page) => hook<number[]>(page, "positions");
    const enabled = (page: Page) => hook<boolean[]>(page, "enabled");

    test("connects and reads all seven joints", async ({ page }) => {
      await open(page);
      await connect(page);
      const names = await hook<string[]>(page, "jointNames");
      expect(names).toHaveLength(7);
      await expect(page.locator("#joints tr")).toHaveCount(7);
      const shown = await tablePositions(page);
      expect(Object.keys(shown)).toEqual(names);
      expect(maxAbsDiff(Object.values(shown), await positions(page))).toBeLessThan(1e-3);
      expect(await enabled(page)).toEqual(new Array(7).fill(false)); // connecting never enables torque
      expect(await hook(page, "protocolErrors")).toEqual([]);
    });

    test("enable holds the present pose", async ({ page }) => {
      await open(page);
      await connect(page);
      const before = await positions(page);
      await enable(page);
      expect(await enabled(page)).toEqual(new Array(7).fill(true));
      await page.waitForTimeout(1000);
      expect(maxAbsDiff(await positions(page), before)).toBeLessThan(1e-3);
      await expect(page.locator("#joints tr .torque")).toHaveText(new Array(7).fill("on"));
    });

    test("a move arrives at its target", async ({ page }) => {
      await open(page);
      await connect(page);
      await enable(page);
      await page.selectOption("#joint", "wrist_flex");
      await page.fill("#delta", "0.3");
      const before = await positions(page);
      await page.click("#move");
      await expect(page.locator("#message")).toContainText("Arrived: wrist_flex", { timeout: 15_000 });
      const target = JSON.parse((await page.locator("#target").textContent())!) as number[];
      // moveTo reports arrival within its tolerance; the hold then settles on the target.
      await expect.poll(async () => maxAbsDiff(await positions(page), target)).toBeLessThan(0.005);
      const after = await positions(page);
      expect(after[3]! - before[3]!).toBeCloseTo(0.3, 2);
    });

    test("the sine wave tracks", async ({ page }) => {
      await open(page);
      await connect(page);
      await enable(page);
      const roll = (await hook<string[]>(page, "jointNames")).indexOf("wrist_roll");
      const start = (await positions(page))[roll]!;
      await page.click("#sine");
      // The simulated wrist roll, sampled in the page while the app streams the sine (2 s period, 2 periods).
      const samples = await page.evaluate(async (i) => {
        const out: number[] = [];
        const until = performance.now() + 3500;
        while (performance.now() < until) {
          out.push(((await (window as any).__mock.positions()) as number[])[i]!);
          await new Promise((r) => setTimeout(r, 20));
        }
        return out;
      }, roll);
      await expect(page.locator("#message")).toContainText("Sine done", { timeout: 15_000 });
      const amplitude = (10 * Math.PI) / 180;
      const seen = (Math.max(...samples) - Math.min(...samples)) / 2;
      expect(Math.abs(seen - amplitude)).toBeLessThan(0.1 * amplitude);
      expect(Math.abs((Math.max(...samples) + Math.min(...samples)) / 2 - start)).toBeLessThan(0.1 * amplitude);
      const reported = Number(await page.locator("#sine").getAttribute("data-amplitude"));
      const error = Number(await page.locator("#sine").getAttribute("data-error"));
      expect(Math.abs(reported - amplitude)).toBeLessThan(0.1 * amplitude);
      // The command reaches the motor within a control period or two; a few degrees would mean it fell behind.
      expect(error).toBeLessThan((2 * Math.PI) / 180);
      expect(Math.abs((await positions(page))[roll]! - start)).toBeLessThan(0.01);
    });

    test("a motor fault puts the arm in FAULT and the UI says why", async ({ page }) => {
      await open(page);
      await connect(page);
      await enable(page);
      await hook(page, "fault", "elbow_flex");
      await expect(page.locator("#state")).toHaveText("fault");
      await expect(page.locator("#fault")).toContainText("elbow_flex");
      await expect(page.locator("#fault")).toContainText("motor fault");
      await expect(page.locator('#joints tr[data-joint="elbow_flex"] .fault')).not.toHaveText("ok");
      await expect(page.locator("#rest")).toBeEnabled();
    });

    test("E-stop turns every motor off", async ({ page }) => {
      await open(page);
      await connect(page);
      await enable(page);
      expect(await enabled(page)).toEqual(new Array(7).fill(true));
      await page.click("#estop");
      await expect(page.locator("#message")).toHaveText("E-stop: torque released on every motor.");
      expect(await enabled(page)).toEqual(new Array(7).fill(false));
      await expect(page.locator("#state")).toHaveText("connected");
    });

    test("Rest returns to the rest pose and releases", async ({ page }) => {
      await open(page);
      await connect(page);
      const rest = await positions(page);
      await enable(page);
      await page.selectOption("#joint", "elbow_flex");
      await page.fill("#delta", "0.25");
      await page.click("#move");
      await expect(page.locator("#message")).toContainText("Arrived: elbow_flex", { timeout: 15_000 });
      expect(maxAbsDiff(await positions(page), rest)).toBeGreaterThan(0.2);
      await page.click("#rest");
      await expect(page.locator("#message")).toHaveText("At rest, torque released.", { timeout: 20_000 });
      expect(maxAbsDiff(await positions(page), rest)).toBeLessThan(0.02);
      expect(await enabled(page)).toEqual(new Array(7).fill(false));
    });

    test("dismissing the chooser leaves the app usable", async ({ page }) => {
      await open(page);
      await hook(page, "setChooser", "dismiss");
      await page.click("#connect");
      await expect(page.locator("#message")).toHaveText("No adapter chosen.");
      await expect(page.locator("#connect")).toBeEnabled();
      await expect(page.locator("#state")).toHaveText("not connected");
      await hook(page, "setChooser", "pick");
      await connect(page);
      expect(await hook(page, "choosers")).toBe(2);
    });

    test("unplugging the adapter mid-run is reported and nothing hangs", async ({ page, guard }) => {
      await open(page);
      await connect(page);
      await enable(page);
      await page.click("#sine");
      await page.waitForTimeout(500);
      await hook(page, "unplug");
      await expect(page.locator("#message")).toContainText("Adapter unplugged; the session was closed", { timeout: 10_000 });
      await expect(page.locator("#state")).toHaveText("adapter unplugged");
      await expect(page.locator("#connect")).toBeEnabled();
      const closedIn = Number((await page.locator("#message").textContent())!.match(/closed in (\d+) ms/)![1]);
      console.log(`${product}: session closed ${closedIn} ms after the unplug`);
      expect(closedIn).toBeLessThan(5000);
      // The sine that was streaming ends with an error rather than hanging.
      await expect(page.locator("#sine")).toBeDisabled();
      // A fresh adapter, plugged in, connects.
      await hook(page, "replug");
      await connect(page);
      await enable(page);
      expect(await enabled(page)).toEqual(new Array(7).fill(true));
      expect(guard.consoleErrors.some((l) => /USB read failed|receive stopped/.test(l))).toBe(true);
    });

    test("a second connect after disconnect works", async ({ page }) => {
      await open(page);
      await connect(page);
      await enable(page);
      await page.click("#disconnect");
      await expect(page.locator("#message")).toHaveText("Disconnected.");
      expect(await enabled(page)).toEqual(new Array(7).fill(false));
      await expect(page.locator("#connect")).toBeEnabled();
      await connect(page);
      expect(maxAbsDiff(Object.values(await tablePositions(page)), await positions(page))).toBeLessThan(1e-3);
      await enable(page);
      expect(await enabled(page)).toEqual(new Array(7).fill(true));
    });
  });
}

